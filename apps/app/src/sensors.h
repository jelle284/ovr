#ifndef SENSORS_H
#define SENSORS_H


#include "ovr.h"
#include "opencv2/opencv.hpp"
#include "ui/ui_elements.h"
using namespace ui;

class UISensors {
public:
	UISensors();
	~UISensors();
	void activate();
	void loop(cv::Mat& canvas, mouse_data_t mouse);
	void deactivate();
private:
	std::vector<ui_element*> ui_elements;
	graph_timeseries graph_imu[3], graph_analogs;
	toggle_button hmd_button, rhc_button, lhc_button;
	radio_buttons device_select_radio;
	EDevice active_device;
	push_button
		push_btn1,
		push_btn2,
		push_btn3,
		push_btn4,

		push_jmid,

		push_jxmin,
		push_jxmax,

		push_jymin,
		push_jymax,

		push_tmin,
		push_tmax;
};

#endif SENSORS_H