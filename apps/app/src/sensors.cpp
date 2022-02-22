#include "sensors.h"


extern DeviceList_t g_Devices;

cv::Rect getScaledRectH(int x, int y, int width, int length, double scale) {
	if (scale < 0) return cv::Rect(x - scale * length, y, -scale * length, width);
	return cv::Rect(x, y, scale * length, width);
}cv::Rect getScaledRectV(int x, int y, int width, int length, double scale) {
	if (scale < 0) return cv::Rect(x, y + scale * length, width, -scale * length);
	return cv::Rect(x, y, width, scale * length);
}

UISensors::UISensors() : 
	graph_imu{
	graph_timeseries(20, 20, 400, 220, 100),
	graph_timeseries(20, 250, 400, 220, 100),
	graph_timeseries(20, 480, 400, 220, 100)
	},
	graph_analogs(440, 20, 400, 220, 100),
	hmd_button(1060, 30, 200, 40, "HMD"),
	rhc_button(1060, 130, 200, 40, "Right Hand"),
	lhc_button(1060, 80, 200, 40, "Left Hand"),
	active_device(EDevice::Hmd)
{
	for (int i = 0; i < 3; ++i) ui_elements.push_back(&graph_imu[i]);
	ui_elements.push_back(&graph_analogs);
	ui_elements.push_back(&hmd_button);
	ui_elements.push_back(&rhc_button);
	ui_elements.push_back(&lhc_button);
	// init radio buttons
	device_select_radio.add(&hmd_button);
	device_select_radio.add(&rhc_button);
	device_select_radio.add(&lhc_button);
}

UISensors::~UISensors()
{
}

void UISensors::activate()
{
}

void UISensors::loop(cv::Mat& canvas, mouse_data_t mouse)
{
	auto dev = g_Devices[static_cast<int>(active_device)];
	using namespace cv;
	// clear canvas
	rectangle(canvas, Rect(0, 0, 1280, 800), Scalar(240, 180, 120), -1);
	// read imu
	
	if (dev->has_data & EDataFlags::IMU) {
		IMUData_t imu_data = dev->get_imu_data();
		graph_imu[0].add(imu_data.gyro(0), 0);
		graph_imu[0].add(imu_data.gyro(1), 1);
		graph_imu[0].add(imu_data.gyro(2), 2);
		graph_imu[1].add(imu_data.accel(0), 0);
		graph_imu[1].add(imu_data.accel(1), 1);
		graph_imu[1].add(imu_data.accel(2), 2);
		graph_imu[2].add(imu_data.mag(0), 0);
		graph_imu[2].add(imu_data.mag(1), 1);
		graph_imu[2].add(imu_data.mag(2), 2);
	}
	// read buttons (if controller)
	if (dev->has_data & EDataFlags::Button) {
		ButtonData_t btn = dev->get_button_data();
		
		graph_analogs.add(btn.joyXY(0), 0);
		graph_analogs.add(btn.joyXY(1), 1);
		graph_analogs.add(btn.trigger, 2);
		const Scalar btn_on(0, 255, 0), btn_off(100, 100, 100);
		rectangle(canvas, Rect(440, 400, 15, 5), (btn.buttons & 1) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(460, 400, 15, 5), (btn.buttons & 2) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(480, 400, 15, 5), (btn.buttons & 4) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(500, 400, 15, 5), (btn.buttons & 8) ? btn_on : btn_off, -1);
	}
	// draw gui elements
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
		if (rhc_button.get_state()) {
			active_device = EDevice::RightHandController;
		}
		if (lhc_button.get_state()) {
			active_device = EDevice::LeftHandController;
		}
	}
}

void UISensors::deactivate()
{
}
