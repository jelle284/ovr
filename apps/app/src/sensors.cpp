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
	graph_timeseries(20, 20, 400, 220, 100, "Gyroscope"),
	graph_timeseries(20, 250, 400, 220, 100, "Accelerometer"),
	graph_timeseries(20, 480, 400, 220, 100, "Magnetometer")
	},
	graph_analogs(440, 20, 400, 220, 100, "Inputs"),
	hmd_button(1060, 30, 200, 40, "HMD"),
	rhc_button(1060, 130, 200, 40, "Right Hand"),
	lhc_button(1060, 80, 200, 40, "Left Hand"),
	active_device(EDevice::Hmd),

	push_btn1(460, 500, 160, 30, "Button 1"),
	push_btn2(460, 540, 160, 30, "Button 2"),
	push_btn3(460, 580, 160, 30, "Button 3"),
	push_btn4(460, 620, 160, 30, "Button 4"),

	push_jmid(660, 500, 160, 30, "Analog Mid"),

	push_jxmin(660, 540, 160, 30, "Jx Min"),
	push_jxmax(660, 580, 160, 30, "Jx Max"),

	push_jymin(660, 620, 160, 30, "Jy Min"),
	push_jymax(860, 500, 160, 30, "Jy Max"),

	push_tmin(860, 540, 160, 30, "Trig Min"),
	push_tmax(860, 580, 160, 30, "Trig Max"),
	logfile_button(880, 30, 140, 30, "Log to file")
{
	for (int i = 0; i < 3; ++i) ui_elements.push_back(&graph_imu[i]);
	ui_elements.push_back(&graph_analogs);
	ui_elements.push_back(&hmd_button);
	ui_elements.push_back(&rhc_button);
	ui_elements.push_back(&lhc_button);

	ui_elements.push_back(&push_btn1);
	ui_elements.push_back(&push_btn2);
	ui_elements.push_back(&push_btn3);
	ui_elements.push_back(&push_btn4);

	ui_elements.push_back(&push_jmid);

	ui_elements.push_back(&push_jxmin);
	ui_elements.push_back(&push_jxmax);

	ui_elements.push_back(&push_jymin);
	ui_elements.push_back(&push_jymax);

	ui_elements.push_back(&push_tmin);
	ui_elements.push_back(&push_tmax);
	
	ui_elements.push_back(&logfile_button);

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

	// read buttons (if controller)
	if (dev->has_data & EDataFlags::Button) {
		ButtonData_t btn = dev->get_button_data();
		
		graph_analogs.add(btn.joyXY(0), 0);
		graph_analogs.add(btn.joyXY(1), 1);
		graph_analogs.add(btn.trigger, 2);
		const Scalar btn_on(0, 255, 0), btn_off(100, 100, 100);
		rectangle(canvas, Rect(440, 400, 15, 5), (btn.buttons == 1) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(460, 400, 15, 5), (btn.buttons == 2) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(480, 400, 15, 5), (btn.buttons == 3) ? btn_on : btn_off, -1);
		rectangle(canvas, Rect(500, 400, 15, 5), (btn.buttons == 4) ? btn_on : btn_off, -1);
	}
	// teach in button values
	auto adc_data = dev->get_adc_data();
	if (push_btn1.get_state()) {
		dev->teachButton(EButton::Btn1, adc_data.btn, 100);
	}
	if (push_btn2.get_state()) {
		dev->teachButton(EButton::Btn2, adc_data.btn, 100);
	}
	if (push_btn3.get_state()) {
		dev->teachButton(EButton::Btn3, adc_data.btn, 100);
	}
	if (push_btn4.get_state()) {
		dev->teachButton(EButton::Btn4, adc_data.btn, 100);
	}
	if (push_jmid.get_state()) {
		dev->teachButton(EButton::JxMid, adc_data.jx, 0);
		dev->teachButton(EButton::JyMid, adc_data.jy, 0);
	}
	if (push_jxmin.get_state()) {
		dev->teachButton(EButton::JxMin, adc_data.jx, 0);
	}
	if (push_jxmax.get_state()) {
		dev->teachButton(EButton::JxMax, adc_data.jx, 0);
	}
	if (push_jymin.get_state()) {
		dev->teachButton(EButton::JyMin, adc_data.jy, 0);
	}
	if (push_jymax.get_state()) {
		dev->teachButton(EButton::JyMax, adc_data.jy, 0);
	}
	if (push_tmin.get_state()) {
		dev->teachButton(EButton::Tmin, adc_data.trig, 0);
	}
	if (push_tmax.get_state()) {
		dev->teachButton(EButton::Tmax, adc_data.trig, 0);
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

	// log file button
	if (logfile_button.has_changed) {
		if (logfile_button.get_state()) {
			logfile.open("sensorlog.csv");
			logfile << "t,ax,ay,az,gx,gy,gz,mx,my,mz\n";
			logfile_t0 = std::chrono::system_clock::now();
		}
		else {
			logfile.close();
		}
	}
	if (logfile_button.get_state()) {
		std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - logfile_t0;
		logfile << elapsed.count() << ","
			<< imu_data.accel(0) << ","
			<< imu_data.accel(1) << ","
			<< imu_data.accel(2) << ","
			<< imu_data.gyro(0) << ","
			<< imu_data.gyro(1) << ","
			<< imu_data.gyro(2) << ","
			<< imu_data.mag(0) << ","
			<< imu_data.mag(1) << ","
			<< imu_data.mag(2) << "\n";
	}
}

void UISensors::deactivate()
{
}
