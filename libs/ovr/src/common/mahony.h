#ifndef MAHONY_H
#define MAHONY_H

#include "ovr.h"

/* Mahony Quaternion update */
Quaternionf mahonyQuaternionUpdate(IMUData_t imu, Quaternionf q);

Quaternionf deadReckon(cv::Matx31f gyro, Quaternionf q);

Quaternionf mahonyFromRef(IMUData_t imu, Quaternionf q, const cv::Matx31f& acc_ref, const cv::Matx31f& mag_ref);

template<typename T>
cv::Matx<T, 3, 1> cross(const cv::Matx<T, 3, 1>& a, const cv::Matx<T, 3, 1>& b) {
    return cv::Matx<T, 3, 1>(
        a(1) * b(2) - a(2) * b(1),
        -a(0) * b(2) + a(2) * b(0),
        a(0) * b(1) - a(1) * b(0));
}
#endif