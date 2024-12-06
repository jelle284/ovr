#include "ovr.h"
#include "ps3eye.h"
#include "camera.h"
#include "hmd.h"
#include "hand_controller.h"
#include "udp_server.h"

HeadMountDisplay g_Hmd;
HandController g_RHC(EDevice::RightHandController), g_LHC(EDevice::LeftHandController);
UDPServer g_udp;

OVR_DLL_EXPORT CameraList_t getCameras()
{
	std::vector<ICamera*> v;
	auto devices = ps3eye::PS3EYECam::getDevices();
	for (auto d : devices) {
		Camera* pCamera = new Camera(d);
		pCamera->registerDevice((TrackerBase*)&g_Hmd);
		pCamera->registerDevice((TrackerBase*)&g_RHC);
		pCamera->registerDevice((TrackerBase*)&g_LHC);
		v.push_back(pCamera);
	}
	return v;
}

OVR_DLL_EXPORT DeviceList_t getDevices()
{
	g_udp.subscribe(&g_RHC);
	g_udp.subscribe(&g_LHC);
	return std::array<IDevice*, 3>({ &g_Hmd, &g_LHC, &g_RHC });
}
