#ifndef TRACKING_H
#define TRACKING_H

#include "ovr.h"
#include "opencv2/opencv.hpp"
#include "ui/ui_elements.h"
#include "ui/viewer.h"

class UITracking {
public:
	UITracking();
	~UITracking();
	void activate();
	void loop(cv::Mat& canvas, mouse_data_t mouse);
	void deactivate();
private:
	const cv::Rect ls, rs, hmdbox, rhcbox, lhcbox;
	toggle_button device_button[3], camera_button[4];
	single_slider process_slider, measurement_slider;
	push_button align_button, clear_button;
	static_text quat_label[3], pos_label[3];

	std::vector<ui_element*> ui_elements;
	Viewer viewer;
	cv::Mat image_buffer;

	CoordinateSystem base_view, camera_view[4];
	Arrow ray_view[4][3], dev_view[3];
	Circle pos_view[3];
	
};
#endif // !TRACKING_H