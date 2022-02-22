#include "ovr.h"
#include "ps3eye.h"
#include "camera/camera.h"
#include "devices/hmd.h"
#include "devices/hand_controller.h"

CAMERA_DLL_EXPORT CameraList_t getCameras()
{
	std::vector<ICamera*> v;
	auto devices = ps3eye::PS3EYECam::getDevices();
	for (auto d : devices) {
		v.push_back(new Camera(d));
	}
	return v;
}

HeadMountDisplay g_Hmd;
HandController g_RHC(EDevice::RightHandController), g_LHC(EDevice::LeftHandController);

CAMERA_DLL_EXPORT DeviceList_t getDevices()
{
	return std::array<IDevice*, 3>({ &g_Hmd, &g_LHC, &g_RHC });
}
