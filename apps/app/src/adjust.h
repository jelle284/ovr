#ifndef ADJUST_H
#define ADJUST_H

#include "ovr.h"
#include "ui/ui_elements.h"

class UIAdjust {
public:
	UIAdjust();
	~UIAdjust();
	void activate();
	void loop(cv::Mat& canvas, mouse_data_t mouse);
	void deactivate();
	
private:
	cv::Mat image_buffer;
	const cv::Rect ls, rs, cp;

	int timer_counter;
	double time_ms;

	toggle_button camera_button[4];
	radio_buttons camera_select_radio;

	EDevice active_device;
	ICamera* active_cam;

	std::vector<ui_element*> ui_elements;
	double_ended_slider hue_slider, sat_slider, value_slider;
	single_slider exposure_slider, gain_slider;
	color_picker target_picker;
	toggle_button auto_gain_button, morph_button, invert_button,
		hmd_button, rhc_button, lhc_button;
	radio_buttons device_select_radio;

	void _to_ui();
	void ui_change();
};

#endif // !ADJUST_H