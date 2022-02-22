#ifndef TRACKING_MATH_H
#define TRACKING_MATH_H

#include "opencv2/opencv.hpp"
#include "ovr.h"

/* Kalman filter */
class KalmanFilter3 {
public:
	KalmanFilter3();
	void predict();
	void correct(cv::Matx31d pos3d);
	void makeAB(double dt);
	void setAcc(cv::Matx31d acc) { u = acc; }
	void setNoise(double process_noise, double measurement_noise) { wp = process_noise; wm = measurement_noise; }

	cv::Matx61d get_state() { return x; }
	cv::Matx31d get_pos() { return x.get_minor<3, 1>(0, 0); }
	cv::Matx31d get_vel() { return x.get_minor<3, 1>(3, 0); }
private:
	cv::Matx61d x, x_; // pos, vel
	cv::Matx66d F, Pk, Pk_;
	cv::Matx<double, 6, 3> B;
	cv::Matx<double, 3, 6> H;
	cv::Matx31d u;
	double wp, wm; // process and measurement noise
};
#endif