
#include "tracking.h"

extern DeviceList_t g_Devices;
extern CameraList_t g_Cameras;
extern double g_ProcesNoise;
extern double g_MeasurementNoise;

const double noise_slider_scale = 10.0;

UITracking::UITracking() :
	image_buffer(480, 640, CV_8UC1),
	ls(0, 0, 960, 720), rs(960, 0, 320, 720),
	hmdbox(1000, 190, 240, 140),
	rhcbox(1000, 340, 240, 140),
	lhcbox(1000, 490, 240, 140),
	camera_button{
		toggle_button(1000, 20, 30, 30, "1"),
		toggle_button(1040, 20, 30, 30, "2"),
		toggle_button(1080, 20, 30, 30, "3"),
		toggle_button(1120, 20, 30, 30, "4")
	},
	process_slider(1000, 80, 240, 20, "Proces Noise", 0, 100),
	measurement_slider(1000, 140, 240, 20, "Measurement Noise", 0, 100),
	device_button{
		toggle_button(1010, 200, 160, 30, "HMD"),
		toggle_button(1010, 350, 160, 30, "Left Hand"),
		toggle_button(1010, 500, 160, 30, "Right Hand"),
	},
	viewer(960,720),
	camera_view{
		CoordinateSystem(0.4,1.0),
		CoordinateSystem(0.4,1.0),
		CoordinateSystem(0.4,1.0),
		CoordinateSystem(0.4,1.0)
	},
	align_button(1000, 640, 80, 40, "Align"),
	clear_button(1090, 640, 80, 40, "Clear"),
	quat_label{
		static_text(1010, 250, 220, 30),
		static_text(1010, 400, 220, 30),
		static_text(1010, 550, 220, 30)
	},
	pos_label{
	static_text(1010, 290, 220, 30),
	static_text(1010, 440, 220, 30),
	static_text(1010, 590, 220, 30)
	}
{
	for (int i = 0; i < 4; ++i) ui_elements.push_back(&camera_button[i]);
	for (int i = 0; i < 3; ++i) ui_elements.push_back(&device_button[i]);

	ui_elements.push_back(&process_slider);
	ui_elements.push_back(&measurement_slider);
	ui_elements.push_back(&align_button);
	ui_elements.push_back(&clear_button);
}

UITracking::~UITracking()
{
}

void UITracking::activate()
{
	for (int i = 0; i < g_Cameras.size(); ++i) {
		auto cal = g_Cameras[i]->get_reverse_calibration();
		camera_view[i].rv = cal.rv;
		camera_view[i].tv = cal.tv;

		if (camera_button[i].get_state()) g_Cameras[i]->start(ECameraRunMode::TrackingAuto);

		for (int j = 0; j < g_Devices.size(); ++j) {
			ray_view[i][j].origin = cal.tv;
		}
	}
	process_slider.set_value(noise_slider_scale * g_ProcesNoise);
	measurement_slider.set_value(noise_slider_scale * g_MeasurementNoise);
}

void UITracking::loop(cv::Mat& canvas, mouse_data_t mouse)
{
	using namespace cv;
	// clear canvas
	rectangle(canvas, ls, Scalar(40, 20, 10), -1);
	rectangle(canvas, rs, Scalar(200, 120, 80), -1);
	rectangle(canvas, hmdbox, Scalar(80, 40, 20), 1);
	rectangle(canvas, rhcbox, Scalar(80, 40, 20), 1);
	rectangle(canvas, lhcbox, Scalar(80, 40, 20), 1);

	// run gui
	for (auto& elem : ui_elements) {
		elem->mouse_handler(mouse);
		elem->draw(canvas);
	}
	// update viewer
	if IN_RECT(mouse.xs, mouse.ys, ls) {
		viewer.mouse_handler(mouse(ls));
	}
	base_view.project(canvas(ls), viewer);

	// get filter slider
	if (process_slider.has_changed || measurement_slider.has_changed) {
		g_ProcesNoise = (double)process_slider.get_value() / noise_slider_scale;
		g_MeasurementNoise = (double)measurement_slider.get_value() / noise_slider_scale;
	}

	// run cameras
	for (int i = 0; i < g_Cameras.size(); ++i) {
		// set tracker status from buttons
		for (int j = 0; j < 3; ++j) {
			g_Cameras[i]->set_tracker_flags((EDevice)(j), ETrackerFlag::Status, device_button[j].get_state());
		}
		// set camera running status
		bool status = camera_button[i].get_state();
		if (camera_button[i].has_changed) {
			status ? g_Cameras[i]->start(ECameraRunMode::TrackingAuto) : g_Cameras[i]->stop();
		}
		if (status) {
			// draw rays
			for (int j = 0; j < 3; ++j) {
				Matx31d ray;
				if (g_Cameras[i]->get_tracker_ray(EDevice(j), ray)) {
					ray_view[i][j].direction = ray;
					ray_view[i][j].project(canvas(ls), viewer);
				}
			}
			// draw cameras on viewer
			camera_view[i].project(canvas(ls), viewer);
		}
	}
	// run devices
	for (int i = 0; i < 3; ++i) {
		// check if button is toggled
		if (!device_button[i].get_state()) continue;

		// get pose from device
		IDevice* pDevice = g_Devices[i];

		pDevice->update_from_ext(g_Cameras);

		auto pose = pDevice->get_pose_data();

		// draw position
		pos_view[i].center(0) = pose.pos(0);
		pos_view[i].center(1) = pose.pos(1);
		pos_view[i].center(2) = pose.pos(2);
				
		// draw rotation
		Quaternionf d(0.0, 0.0, 0.0, -0.2);
		Quaternionf dn = pose.q * d * pose.q.conjugate();
		dev_view[i].origin = pos_view[i].center;
		dev_view[i].direction = dn.imag();

		{	// print quaternion
			std::stringstream ss;
			ss.precision(2);
			ss << std::fixed << "quat: " << pose.q.val[0] << ", " << pose.q.val[1] << ", " << pose.q.val[2] << ", " << pose.q.val[3];
			quat_label[i].text = ss.str();
		}
		{	// print position
			std::stringstream ss;
			ss.precision(2);
			ss << std::fixed << "pos: " << pose.pos(0) << ", " << pose.pos(1) << ", " << pose.pos(2);
			pos_label[i].text = ss.str();
		}
		// draw components onto canvas
		quat_label[i].draw(canvas);
		pos_label[i].draw(canvas);
		pos_view[i].project(canvas(ls), viewer);
		dev_view[i].project(canvas(ls), viewer);
	}
	if (align_button.get_state() && align_button.has_changed) {
		for (auto& d : g_Devices) {
			d->align();
		}
	}
	if (clear_button.get_state()) {
		for (auto& d : g_Devices) {
			//TODO: clear
		}
	}
}

void UITracking::deactivate()
{
	for (auto c : g_Cameras) {
		c->stop();
	}
}