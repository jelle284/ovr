#include "adjust.h"
#include "calibrate.h"
#include "tracking.h"
#include "sensors.h"
#include <Windows.h>
#include "ovr.h"


using namespace cv;

/* Globals */
auto g_Cameras = getCameras();
auto g_Devices = getDevices();

double g_ProcesNoise;
double g_MeasurementNoise;

/* App Main */
void app() {
	mouse_data_t mouse;
	std::string winname = "ovr app";
	namedWindow(winname);
	setMouseCallback(winname, on_mouse, &mouse);
	Mat canvas = Mat::zeros(800, 1280, CV_8UC3);

	// top bar
	toggle_button adjust_button(10, 10, 280, 60, "Adjust"),
		tracking_button(300, 10, 280, 60, "View"),
		calib_button(590, 10, 280, 60, "Calibrate"),
		sensors_button(890, 10, 280, 60, "Sensors");
	radio_buttons top_bar_radio;
	top_bar_radio.add(&adjust_button);
	top_bar_radio.add(&tracking_button);
	top_bar_radio.add(&calib_button);
	top_bar_radio.add(&sensors_button);
	toggle_button* top_bar_last_btn = top_bar_radio.get_current_btn();

	// content window
	Rect content(0, 80, 1280, 720);

	// content pages
	UIAdjust ui_adjust;
	UICalibrate ui_calib;
	UITracking ui_tracking;
	UISensors ui_sensors;

	{	// load config
		cv::FileStorage fs("vr_calib.json", cv::FileStorage::READ);
		for (auto c : g_Cameras) fs[c->file_id()] >> c;
		for (auto d : g_Devices) fs[d->get_name()] >> d;

		fs["Proces Noise"] >> g_ProcesNoise;
		fs["Measurement Noise"] >> g_MeasurementNoise;
	}
	


	// tracked devices
	for (auto& d : g_Devices) d->start();

	bool running = true;
	ui_adjust.activate();

	while (running) {
		// render top bar
		adjust_button.draw(canvas);
		adjust_button.mouse_handler(mouse);
		tracking_button.draw(canvas);
		tracking_button.mouse_handler(mouse);
		calib_button.draw(canvas);
		calib_button.mouse_handler(mouse);
		sensors_button.draw(canvas);
		sensors_button.mouse_handler(mouse);
		top_bar_radio.update();

		// change page
		if (top_bar_radio.has_changed) {
			// deactivate last
			if (top_bar_last_btn == &adjust_button) ui_adjust.deactivate();
			if (top_bar_last_btn == &calib_button) ui_calib.deactivate();
			if (top_bar_last_btn == &tracking_button) ui_tracking.deactivate();
			// activate new
			if (adjust_button.get_state()) ui_adjust.activate();
			if (calib_button.get_state()) ui_calib.activate();
			if (tracking_button.get_state()) ui_tracking.activate();
			// update last
			top_bar_last_btn = top_bar_radio.get_current_btn();
		}
		
		// render content
		if (adjust_button.get_state())
			ui_adjust.loop(canvas(content), mouse(content));
		if (calib_button.get_state())
			ui_calib.loop(canvas(content), mouse(content));
		if (tracking_button.get_state())
			ui_tracking.loop(canvas(content), mouse(content));
		if (sensors_button.get_state())
			ui_sensors.loop(canvas(content), mouse(content));

		// show images
		imshow(winname, canvas);
		int key = cv::waitKey(10);
		if WIN_CLOSED(winname) running = false;
	}
	cv::destroyAllWindows();

	for (auto d : g_Devices) d->stop();
	for (auto c : g_Cameras) c->stop();

	{	// save config
		cv::FileStorage fs("vr_calib.json", cv::FileStorage::WRITE);
		for (auto c : g_Cameras) fs << c->file_id() << c;
		for (auto d : g_Devices) fs << d->get_name() << d;

		fs << "Proces Noise" << g_ProcesNoise;
		fs << "Measurement Noise" << g_MeasurementNoise;
	}
}

/* Run app throgh win32 API */
int WINAPI wWinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPWSTR, _In_ int) {
	app();
	return 0;
}

/* Run app as console */
int main(int argc, char* argv[]) {
	app();
	return 0;
}