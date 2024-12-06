#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <cassert>
#include "opencv2/opencv.hpp"

/* Quaternion template class */

template <typename T>
class Quaternion_ {
public:
	T val[4];
	Quaternion_() : val{ 1.0, 0.0, 0.0, 0.0 } {}
	Quaternion_(T w, T x, T y, T z) : val{ w, x, y, z } {}
	Quaternion_(cv::Matx<T, 3, 1> m) : val{ T(0), m(0), m(1), m(2) } {}
	Quaternion_(cv::Matx<T, 4, 1> m) : val{ m(0), m(1), m(2), m(3) } {}

	T operator()(int n) const {
		assert(n >= 0 && n < 4 && "Index out of bounds");
		return val[n];
	}

	Quaternion_ operator* (const Quaternion_& q) const {
		return Quaternion_(
			val[0] * q.val[0] - val[1] * q.val[1] - val[2] * q.val[2] - val[3] * q.val[3],
			val[0] * q.val[1] + val[1] * q.val[0] + val[2] * q.val[3] - val[3] * q.val[2],
			val[0] * q.val[2] - val[1] * q.val[3] + val[2] * q.val[0] + val[3] * q.val[1],
			val[0] * q.val[3] + val[1] * q.val[2] - val[2] * q.val[1] + val[3] * q.val[0]);
	}
	Quaternion_ operator* (const T scalar) const {
		return Quaternion_(
			scalar * val[0],
			scalar * val[1],
			scalar * val[2], 
			scalar * val[3]);
	}
	Quaternion_ operator+ (const Quaternion_& q) const {
		return Quaternion_(
			val[0] + q.val[0],
			val[1] + q.val[1],
			val[2] + q.val[2],
			val[3] + q.val[3]);
	}
	Quaternion_ operator- (const Quaternion_& q) const {
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
			1 - 2 * (val[2] * val[2] + val[3] * val[3]),
			2 * (val[1] * val[2] - val[0] * val[3]),
			2 * (val[1] * val[3] + val[0] * val[2]),

			2 * (val[1] * val[2] + val[0] * val[3]),
			1 - 2 * (val[1] * val[1] + val[3] * val[3]),
			2 * (val[2] * val[3] - val[0] * val[1]),

			2 * (val[1] * val[3] - val[0] * val[2]),
			2 * (val[2] * val[3] + val[0] * val[1]),
			1 - 2 * (val[1] * val[1] + val[2] * val[2])
		);
	}
};

template <typename T>
Quaternion_<T> operator* (T scalar, const Quaternion_<T>& q) {
	return q * scalar;
}

typedef Quaternion_<double> Quaterniond;
typedef Quaternion_<float> Quaternionf;

/* 3D vector cross product */
template<typename T>
cv::Matx<T, 3, 1> cross(const cv::Matx<T, 3, 1>& a, const cv::Matx<T, 3, 1>& b) {
	return cv::Matx<T, 3, 1>(
		a(1) * b(2) - a(2) * b(1),
		-a(0) * b(2) + a(2) * b(0),
		a(0) * b(1) - a(1) * b(0));
}

#endif

