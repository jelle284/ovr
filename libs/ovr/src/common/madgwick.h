#ifndef MADGWICK_H
#define MADGWICK_H

#include "ovr.h"

Quaternionf madgwickFilterUpdate(
	IMUData_t IMUData,
	Quaternionf q,
	const cv::Matx31f acc_ref,
	const cv::Matx31f mag_reg,
	const float beta = 0.1f
	);

#endif