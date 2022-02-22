#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "common.h"

//-----------------------------------------------------------------------------
// Purpose: Hand Controller
//-----------------------------------------------------------------------------
class COvrControllerDriver : 
	public DriverTrackedDevice
{
public:
	COvrControllerDriver(vr::ETrackedControllerRole role);

	virtual ~COvrControllerDriver();

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);

	virtual void Deactivate();

	virtual void EnterStandby();

	void* GetComponent(const char* pchComponentNameAndVersion);

	virtual void PowerOff();

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);

	virtual void UpdateButtons(ButtonData_t btn) override;
private:
	VRInputComponentHandle_t m_compA;
	VRInputComponentHandle_t m_compB;
	VRInputComponentHandle_t m_compGrip;
	VRInputComponentHandle_t m_compJoystickClick;
	VRInputComponentHandle_t m_compJoystickX;
	VRInputComponentHandle_t m_compJoystickY;
	VRInputComponentHandle_t m_compTrigger;

	ETrackedControllerRole m_role;
};

#endif // !CONTROLLER_H