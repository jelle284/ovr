#ifndef BASE_STATION_H
#define BASE_STATION_H

#include "common.h"

//-----------------------------------------------------------------------------
// Purpose: Base Station
//-----------------------------------------------------------------------------
class COvrBaseStation : 
	public DriverTrackedDevice
{
public:
	COvrBaseStation();

	~COvrBaseStation();

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);

	virtual void Deactivate();
	
	virtual void EnterStandby();

	void* GetComponent(const char* pchComponentNameAndVersion);

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);
};

#endif // !BASE_STATION_H