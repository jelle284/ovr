#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "ovr.h"
#include "opencv2/opencv.hpp"
#include "ui/ui_elements.h"
using namespace ui;

class UICalibrate {
public:

	UICalibrate();
	~UICalibrate();
	void activate();
	void loop(cv::Mat& canvas, mouse_data_t mouse);
	void deactivate();

private:
	std::vector<std::vector<cv::Point2f>> intrinsic_points;
	std::chrono::system_clock::time_point t_capture;
	bool instrinsic_capturing;
	
	cv::Mat image_buffer;
	const cv::Rect ls, rs, cp;
	int mode;
	toggle_button intrinsics_button, extrinsics_button;
	radio_buttons calib_mode_radio;
	single_slider npoints_slider;
	
	toggle_button camera_button[4];
	radio_buttons camera_select_radio;

	ICamera* active_cam;

	push_button start_button;

	bool found = false;
	std::vector<cv::Point2f> chess_points;
};

enum class ECalibAngle {
	front,
	left,
	right
};

std::vector<cv::Point3d> chesspattern3d(double chess_size,
	ECalibAngle calib_angle = ECalibAngle::front);

std::vector<cv::Point3f> chesspattern3f(float chess_size,
	ECalibAngle calib_angle = ECalibAngle::front);

#endif // !CALIBRATE_H
