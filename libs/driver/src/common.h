#ifndef COMMON_H
#define COMMON_H

#include <openvr_driver.h>
#include "driverlog.h"

#include "ovr.h"

#include <thread>
#include <chrono>

using namespace vr;

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

class DriverTrackedDevice
	: public ITrackedDeviceServerDriver 
{
protected:
	TrackedDeviceIndex_t m_unObjectId;
	PropertyContainerHandle_t m_ulPropertyContainer;
	DriverPose_t m_Pose;
public:
	DriverTrackedDevice() {
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

		m_Pose = { 0 };
		m_Pose.poseIsValid = false;
		m_Pose.result = TrackingResult_Uninitialized;
		m_Pose.deviceIsConnected = false;
		m_Pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		m_Pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);
	}
	virtual DriverPose_t GetPose() override { return m_Pose; }
	void UpdatePose(DriverPose_t DriverPose) {
		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
		{
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(m_unObjectId, DriverPose, sizeof(DriverPose_t));
		}
	}
	virtual void UpdateButtons(ButtonData_t btn) {};
	virtual void RunFrame() {}
	TrackedDeviceIndex_t getUniqueObjectID() { return m_unObjectId; }
};
#endif // !COMMON_H