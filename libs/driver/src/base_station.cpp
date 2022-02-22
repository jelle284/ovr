#include "base_station.h"

COvrBaseStation::COvrBaseStation()
{
	DriverLog("COvrBaseStation: Constructor\n");
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
}

COvrBaseStation::~COvrBaseStation()
{
}

EVRInitError COvrBaseStation::Activate(vr::TrackedDeviceIndex_t unObjectId)
{
	DriverLog("COvrBaseStation: Activate\n");
	m_unObjectId = unObjectId;
	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

	// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);
	
	return VRInitError_None;
}

void COvrBaseStation::Deactivate()
{
	DriverLog("COvrBaseStation: Deactivate entry\n");
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	DriverLog("COvrBaseStation: Deactivate exit\n");
}

void COvrBaseStation::EnterStandby()
{
}

void *COvrBaseStation::GetComponent(const char* pchComponentNameAndVersion)
{
	// override this to add a component to a driver
	return NULL;
}

void COvrBaseStation::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if (unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}