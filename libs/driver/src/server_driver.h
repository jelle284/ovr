#ifndef SERVER_DRIVER_H
#define SERVER_DRIVER_H

#include "hmd.h"
#include "controller.h"
#include "base_station.h"
#include "ovr.h"

//-----------------------------------------------------------------------------
// Purpose: Server Driver
//-----------------------------------------------------------------------------
class COvrServerDriver: public IServerTrackedDeviceProvider
{
public:

	virtual EVRInitError Init(vr::IVRDriverContext *pDriverContext) ;
	virtual void Cleanup() ;
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame() ;
	virtual bool ShouldBlockStandbyMode()  { return false; }
	virtual void EnterStandby()  {}
	virtual void LeaveStandby()  {}

private:
	DeviceList_t DeviceList;
	CameraList_t CameraList;

	COvrHMDDriver *m_pHMD = nullptr;
	COvrControllerDriver *m_pLHController = nullptr;
	COvrControllerDriver *m_pRHController = nullptr;
	std::vector<COvrBaseStation*> m_pBaseStations;
	std::thread *m_pTrackingThread = nullptr;
	bool m_TrackingThreadState = false;

	void threadFunc();
};

#endif // !SERVER_DRIVER_H