#include "adjust.h"


extern CameraList_t g_Cameras;

inline cv::Vec2b slider_bounds(double_ended_slider& sld) {
	return cv::Vec2b(sld.get_lower(), sld.get_upper());
}

UIAdjust::UIAdjust() :
	image_buffer(480, 640, CV_8UC1),
	ls(0, 240, 640, 480),
	rs(640, 240, 640, 480),
	cp(0, 0, 1280, 240),
	hue_slider(40, 30, 280, 30, "Hue", 0, 179), invert_button(340, 30, 100, 30, "Invert"),
	sat_slider(40, 100, 280, 30, "Saturation", 0, 255),
	value_slider(40, 170, 280, 30, "Value", 0, 255),
	target_picker(460, 140, 60, ls),
	exposure_slider(600, 30, 260, 30, "Exposure", 0, 255),
	gain_slider(600, 100, 260, 30, "Gain", 0, 63),
	auto_gain_button(880, 60, 80, 40, "Auto"),
	morph_button(760, 180, 100, 30, "Morph"),
	hmd_button(1060, 30, 200, 40, "HMD"),
	rhc_button(1060, 130, 200, 40, "Right Hand"),
	lhc_button(1060, 80, 200, 40, "Left Hand"),
	camera_button{
		toggle_button(600, 180, 30, 30, "1"),
		toggle_button(640, 180, 30, 30, "2"),
		toggle_button(680, 180, 30, 30, "3"),
		toggle_button(720, 180, 30, 30, "4") },
	active_cam(nullptr),
	active_device(EDevice::Hmd),
	time_ms(0), timer_counter(0)
{
	// init radio buttons
	device_select_radio.add(&hmd_button);
	device_select_radio.add(&rhc_button);
	device_select_radio.add(&lhc_button);
	for (int i = 0; i < 4; ++i) camera_select_radio.add(&camera_button[i]);

	// collect ui elements in vector
	ui_elements.push_back(&value_slider);
	ui_elements.push_back(&sat_slider);
	ui_elements.push_back(&hue_slider);
	ui_elements.push_back(&invert_button);
	ui_elements.push_back(&target_picker);
	for (int i = 0; i < 4; ++i) ui_elements.push_back(&camera_button[i]);
	ui_elements.push_back(&exposure_slider);
	ui_elements.push_back(&gain_slider);
	ui_elements.push_back(&auto_gain_button);
	ui_elements.push_back(&morph_button);
	ui_elements.push_back(&hmd_button);
	ui_elements.push_back(&rhc_button);
	ui_elements.push_back(&lhc_button);
}

UIAdjust::~UIAdjust()
{
}

void UIAdjust::activate()
{
	if (!(g_Cameras.empty() && active_cam)) {
		active_cam = g_Cameras.front();
	}
	// load in tracker values
	if (active_cam) {
		_to_ui();
		active_cam->start(ECameraRunMode::TrackingManual);
	}
}

void UIAdjust::loop(cv::Mat& canvas, mouse_data_t mouse)
{
	using namespace cv;
	// clear control panel
	rectangle(canvas, cp, Scalar(240, 180, 120), -1);

	// grab image
	if (active_cam) {
		active_cam->view(canvas(ls));
		
		// init timing
		auto before = std::chrono::system_clock::now();
		// detect
		active_cam->man_tracker_update(active_device, canvas(ls));
		// get timing
		auto elapsed = std::chrono::system_clock::now() - before;
		int time_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();

		// draw mask and ROI
		rectangle(canvas, rs, Scalar(0, 0, 0), -1);
		Rect roi = active_cam->get_tracker_roi(active_device);
		Mat drawZone = canvas(rs)(roi);
		Mat mask = active_cam->get_tracker_mask(active_device);
		cvtColor(mask, drawZone, COLOR_GRAY2BGR);
		rectangle(canvas(ls), roi, Scalar(255, 155, 55), 1);
		
		// display timing every 10th frame
		++timer_counter;
		if (timer_counter > 10) {
			time_ms = (double)(time_us) / 1000;
			timer_counter = 0;
		}
		std::stringstream ss;
		ss.precision(3);
		ss << "Tracking loop time: " << time_ms << " ms";
		putText(canvas, ss.str(), Point(900, 210), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0));

		// detect changes in ui elements
		ui_change();
	}
	else // no camera
	{
		rectangle(canvas, ls, Scalar(0, 0, 0), -1);
		rectangle(canvas, rs, Scalar(0, 0, 0), -1);
		putText(canvas, "No camera", Point(600, 480), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(50, 50, 255));
	}

	// run gui
	for (auto& elem : ui_elements) {
 		elem->mouse_handler(mouse);
		elem->draw(canvas);
	}

	// device selection
	device_select_radio.update();
	if (device_select_radio.has_changed) {
		if (hmd_button.get_state()) {
			active_device = EDevice::Hmd;
		}
		if (lhc_button.get_state()) {
			active_device = EDevice::LeftHandController;
		}
		if (rhc_button.get_state()) {
			active_device = EDevice::RightHandController;
		}
	}

	// camera selection
	camera_select_radio.update();
	if (camera_select_radio.has_changed) {
		if (active_cam) active_cam->stop();
		for (int i = 0; i < 4; ++i) {
			if (camera_button[i].get_state()) {
				active_cam = (i < g_Cameras.size()) ? g_Cameras[i] : nullptr;
			}
		}
		if (active_cam) active_cam->start(ECameraRunMode::TrackingManual);
	}

	// update ui if new camera or new device is selected
	if (device_select_radio.has_changed || camera_select_radio.has_changed) {
		if(active_cam) _to_ui();
	}
}


void UIAdjust::deactivate()
{
	if (active_cam) {
		active_cam->stop();
	}
}

void UIAdjust::_to_ui()
{
	invert_button.set_state(
		active_cam->get_tracker_flags(active_device, ETrackerFlag::HueInvert)
	);
	morph_button.set_state(
		active_cam->get_tracker_flags(active_device, ETrackerFlag::Morph)
	);
	cv::Vec2b bound;
	bound = active_cam->get_tracker_bound(active_device, ETrackerBound::Hue);
	hue_slider.set_values(bound(0), bound(1));

	bound = active_cam->get_tracker_bound(active_device, ETrackerBound::Sat);
	sat_slider.set_values(bound(0), bound(1));

	bound = active_cam->get_tracker_bound(active_device, ETrackerBound::Value);
	value_slider.set_values(bound(0), bound(1));

	auto_gain_button.set_state(active_cam->get_autogain());
	gain_slider.set_value(active_cam->get_gain());
	exposure_slider.set_value(active_cam->get_exposure());
}

void UIAdjust::ui_change()
{
	// update tracker
	if (hue_slider.has_changed) {
		active_cam->set_tracker_bound(active_device, ETrackerBound::Hue, slider_bounds(hue_slider));
	}
	if (sat_slider.has_changed) {
		active_cam->set_tracker_bound(active_device, ETrackerBound::Sat, slider_bounds(sat_slider));
	}
	if (value_slider.has_changed) {
		active_cam->set_tracker_bound(active_device, ETrackerBound::Value, slider_bounds(value_slider));
	}
	if (morph_button.has_changed) {
		active_cam->set_tracker_flags(active_device, ETrackerFlag::Morph, morph_button.get_state());
	}
	if (invert_button.has_changed) {
		active_cam->set_tracker_flags(active_device, ETrackerFlag::HueInvert, invert_button.get_state());
	}
	// update camera
	if (auto_gain_button.has_changed) {
		active_cam->set_autogain(auto_gain_button.get_state());
	}
	if (gain_slider.has_changed) {
		active_cam->set_gain(gain_slider.get_value());
	}
	if (exposure_slider.has_changed) {
		active_cam->set_exposure(exposure_slider.get_value());
	}
}