#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <chrono>

#include "tracking_math.h"
#include "camera.h"
#include "tracked_device.h"

class SensorFusion : public KalmanFilter3 {
public:
	Quaterniond q, qzero;
	SensorFusion();
	~SensorFusion();
	void camUpdate(std::vector<Camera*> cameras, EDeviceTag tag);
	void imuUpdate(imu_update_t imu_data);
private:
	std::chrono::system_clock::time_point t_imu_last;
};

#endif