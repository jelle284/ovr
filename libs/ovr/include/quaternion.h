#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include "opencv2/opencv.hpp"

template <typename T>
class Quaternion_ {
public:
	T val[4];
	Quaternion_() : val{ 1.0, 0.0, 0.0, 0.0 } {}
	Quaternion_(T w, T x, T y, T z) : val{ w, x, y, z } {}
	Quaternion_(cv::Matx<T, 3, 1> m) : val{ T(0), m(0), m(1), m(2) } {}
	Quaternion_(cv::Matx<T, 4, 1> m) : val{ m(0), m(1), m(2), m(3) } {}
	T operator()(int n) const { return val[n]; }
	Quaternion_ operator* (const Quaternion_& q) {
		return Quaternion_(
			val[0] * q.val[0] - val[1] * q.val[1] - val[2] * q.val[2] - val[3] * q.val[3],
			val[0] * q.val[1] + val[1] * q.val[0] + val[2] * q.val[3] - val[3] * q.val[2],
			val[0] * q.val[2] - val[1] * q.val[3] + val[2] * q.val[0] + val[3] * q.val[1],
			val[0] * q.val[3] + val[1] * q.val[2] - val[2] * q.val[1] + val[3] * q.val[0]);
	}
	Quaternion_ operator+ (const Quaternion_& q) {
		return Quaternion_(
			val[0] + q.val[0],
			val[1] + q.val[1],
			val[2] + q.val[2],
			val[3] + q.val[3]);
	}
	Quaternion_ operator- (const Quaternion_& q) {
		return Quaternion_(
			val[0] - q.val[0],
			val[1] - q.val[1],
			val[2] - q.val[2],
			val[3] - q.val[3]);
	}
	T real() { 
		return val[0];
	}
	cv::Matx<T, 3, 1> imag() { 
		return cv::Matx<T, 3, 1>(val[1], val[2], val[3]);
	}
	Quaternion_ add(T s) {
		return Quaternion_(val[0] + s, val[1] + s, val[2] + s, val[3] + s);
	}
	Quaternion_ mul(T s) {
		return Quaternion_(val[0] * s, val[1] * s, val[2] * s, val[3] * s);
	}
	Quaternion_ conjugate() {
		return Quaternion_(val[0], -val[1], -val[2], -val[3]);
	}
	T norm() {
		return std::sqrt(val[0] * val[0] + val[1] * val[1] + val[2] * val[2] + val[3] * val[3]);
	}
	Quaternion_ normalized() {
		T n = norm();
		return Quaternion_(val[0] / n, val[1] / n, val[2] / n, val[3] / n);
	}
	cv::Matx<T, 3, 3> toRotationMatrix() {
		return cv::Matx<T, 3, 3>(
			1 - 2 * val[2] * val[2] - 2 * val[3] * val[3],
			-2 * val[0] * val[3] + 2 * val[1] * val[2],
			2 * val[0] * val[2] + 2 * val[1] * val[3],
			2 * val[0] * val[3] + 2 * val[1] * val[2],
			1 - 2 * val[1] * val[1] - 2 * val[3] * val[3],
			-2 * val[0] * val[1] + 2 * val[2] * val[3],
			-2 * val[0] * val[2] + 2 * val[1] * val[3],
			2 * val[0] * val[1] + 2 * val[2] * val[3],
			1 - 2 * val[1] * val[1] - 2 * val[2] * val[2]
		);
	}
};

typedef Quaternion_<double> Quaterniond;
typedef Quaternion_<float> Quaternionf;

#endif

