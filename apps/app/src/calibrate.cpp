#include "calibrate.h"

extern CameraList_t g_Cameras;

UICalibrate::UICalibrate() :
	image_buffer(480, 640, CV_8UC1),
	cp(0, 0, 1280, 240),
	rs(640, 240, 640, 480),
	ls(0, 240, 640, 480),
	intrinsics_button(10, 10, 200, 60, "Intrinsics"),
	extrinsics_button(300, 10, 200, 60, "Extrinsics"),
	npoints_slider(10, 120, 200, 40, "Points", 0, 60),
	start_button(800, 10, 260, 60, "Start Calibration"),
	camera_button{
		toggle_button(600, 100, 30, 30, "1"),
		toggle_button(640, 100, 30, 30, "2"),
		toggle_button(680, 100, 30, 30, "3"),
		toggle_button(720, 100, 30, 30, "4") },
		active_cam(nullptr),
		mode(1),
		instrinsic_capturing(false), found(false)
{
	calib_mode_radio.add(&intrinsics_button);
	calib_mode_radio.add(&extrinsics_button);
	npoints_slider.set_value(30);
	for (int i = 0; i < 4; ++i) {
		camera_select_radio.add(&camera_button[i]);
	}
}

UICalibrate::~UICalibrate()
{
}

void UICalibrate::activate()
{
	if (!(g_Cameras.empty() && active_cam)) {
		active_cam = g_Cameras.front();
	}
	if (active_cam) {
		active_cam->start(ECameraRunMode::Viewing);
		t_capture = std::chrono::system_clock::now();
	}
}

void UICalibrate::loop(cv::Mat& canvas, mouse_data_t mouse)
{
	using namespace cv;
	// clear control panel
	rectangle(canvas, cp, Scalar(240, 180, 120), -1);
	// clear side bar
	rectangle(canvas, rs, Scalar(200, 120, 80), -1);


	// grab image
	if (active_cam) {
		// get calibration
		auto cal = active_cam->get_calibration();
		// view image from cam
		active_cam->view(canvas(ls));
		// find chessboard
		auto elapsed = std::chrono::system_clock::now() - t_capture;
		if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() > 5) {
			found = findChessboardCorners(canvas(ls), Size(6, 9), chess_points,
				CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);
			t_capture = std::chrono::system_clock::now();
		}
		// intrinsic mode
		if (mode == 1) {
			if (start_button.has_changed && start_button.get_state()) {
				instrinsic_capturing = true;
			}
			if (instrinsic_capturing) {
				if (found) intrinsic_points.push_back(chess_points);
				int npoints = npoints_slider.get_value();
				if (intrinsic_points.size() == npoints) {
					std::vector<std::vector<cv::Point3f>> object_points;
					std::vector<cv::Mat> rvecs;
					std::vector<cv::Mat> tvecs;
					for (int i = 0; i < npoints; ++i) {
						object_points.push_back(chesspattern3f(0.025f));
					}
					Matx33d cm;
					Matx<double, 5, 1> dc;
					double rms = calibrateCamera(
						object_points, intrinsic_points,
						Size(canvas(ls).cols, canvas(ls).rows),
						cm,
						dc,
						rvecs, tvecs);
					active_cam->set_intrinsics(cm, dc);
					intrinsic_points.clear();
					instrinsic_capturing = false;
				}
				putText(canvas(rs),
					"Capturing: " + std::to_string(intrinsic_points.size()),
					Point(canvas(rs).cols / 3, canvas(rs).rows / 3),
					FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 1);
			}
			else {
				putText(canvas(rs),
					"Ready to start calibration",
					Point(canvas(rs).cols / 3, canvas(rs).rows / 3),
					FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 0, 0), 1);
			}
			npoints_slider.draw(canvas);
			npoints_slider.mouse_handler(mouse);
		}
		// extrinsic mode
		if (mode == 2) {
			std::vector<cv::Point3d> model_points = chesspattern3d(0.025);

			if (found) {
				Mat r, t; // temp rotation and translation to show on screen

				cv::solvePnP(
					model_points, chess_points,
					cal.cm,
					cal.dc,
					r, t,
					false);
				std::vector<Point2d> imAxes;
				std::vector<Point3d> worldAxes;
				double unit_length = 0.15;
				worldAxes.push_back(Point3d(0, 0, 0));
				worldAxes.push_back(Point3d(unit_length, 0, 0));
				worldAxes.push_back(Point3d(0, unit_length, 0));
				worldAxes.push_back(Point3d(0, 0, unit_length));

				
				cv::projectPoints(
					worldAxes,
					r, t,
					cal.cm, 
					cal.dc,
					imAxes);

				cv::arrowedLine(canvas(ls), imAxes.at(0), imAxes.at(1), CV_RGB(255, 0, 0), 2);
				cv::putText(canvas(ls), "X", imAxes.at(1), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

				cv::arrowedLine(canvas(ls), imAxes.at(0), imAxes.at(2), CV_RGB(255, 0, 0), 2);
				cv::putText(canvas(ls), "Y", imAxes.at(2), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

				cv::arrowedLine(canvas(ls), imAxes.at(0), imAxes.at(3), CV_RGB(255, 0, 0), 2);
				cv::putText(canvas(ls), "Z", imAxes.at(3), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2);

				if (start_button.has_changed && start_button.get_state()) {
					active_cam->set_extrinsics(r, t);
					std::cout << "Extrinsics saved!" << std::endl;
				}
			}
		}

		extrinsics_button.draw(canvas);
		extrinsics_button.mouse_handler(mouse);
		intrinsics_button.draw(canvas);
		intrinsics_button.mouse_handler(mouse);
		calib_mode_radio.update();
		start_button.draw(canvas);
		start_button.mouse_handler(mouse);
		if (calib_mode_radio.has_changed) {
			if (intrinsics_button.get_state()) mode = 1;
			if (extrinsics_button.get_state()) mode = 2;
		}

		// draw calibration info
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				int s = 50;
				std::stringstream ss;
				ss.precision(3);
				ss << cal.cm(j, i);
				putText(canvas(rs),
					ss.str(),
					Point(canvas(rs).cols / 3 + s * j, canvas(rs).rows / 3 + s * i + 20),
					FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 0, 0), 1);
			}
		}

	}
	else // no camera
	{
		rectangle(canvas, ls, Scalar(0, 0, 0), -1);
		putText(canvas, "No camera", Point(200, 480), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(50, 50, 255));
	}
	for (int i = 0; i < 4; ++i) {
		camera_button[i].draw(canvas);
		camera_button[i].mouse_handler(mouse);
	}
	camera_select_radio.update();
	if (camera_select_radio.has_changed) {
		if (active_cam) active_cam->stop();
		for (int i = 0; i < 4; ++i) {
			if (camera_button[i].get_state()) {
				active_cam = (i < g_Cameras.size()) ? g_Cameras[i] : nullptr;
			}
		}
		if (active_cam) active_cam->start(ECameraRunMode::Viewing);
	}
}

void UICalibrate::deactivate()
{
	if (active_cam) active_cam->stop();
}

std::vector<cv::Point3d> chesspattern3d(double chess_size, ECalibAngle calib_angle)
{
	std::vector<cv::Point3d> chess_pattern;
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 6; j++) {
			switch (calib_angle) {
			case ECalibAngle::front:
				chess_pattern.push_back(cv::Point3d(chess_size * j, chess_size * i, 0));
				break;
			case ECalibAngle::left:
				chess_pattern.push_back(cv::Point3d(0, chess_size * i, chess_size * j));
				break;
			case ECalibAngle::right:
				chess_pattern.push_back(cv::Point3d(0, chess_size * i, -chess_size * i));
				break;
			}
		}
	}
	return chess_pattern;
}

std::vector<cv::Point3f> chesspattern3f(float chess_size, ECalibAngle calib_angle)
{
	std::vector<cv::Point3f> chess_pattern;
	for (int i = 0; i < 9; i++) {
		for (int j = 0; j < 6; j++) {
			switch (calib_angle) {
			case ECalibAngle::front:
				chess_pattern.push_back(cv::Point3f(chess_size * j, chess_size * i, 0));
				break;
			case ECalibAngle::left:
				chess_pattern.push_back(cv::Point3f(0, chess_size * i, chess_size * j));
				break;
			case ECalibAngle::right:
				chess_pattern.push_back(cv::Point3f(0, chess_size * i, -chess_size * i));
				break;
			}
		}
	}
	return chess_pattern;
}


