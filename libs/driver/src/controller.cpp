#include "controller.h"

COvrControllerDriver::COvrControllerDriver(vr::ETrackedControllerRole role) :
	m_role(role)
{
	DriverLog("COvrControllerDriver: Constructor\n");

}

COvrControllerDriver::~COvrControllerDriver()
{
}

EVRInitError COvrControllerDriver::Activate( vr::TrackedDeviceIndex_t unObjectId )
{
	DriverLog("COvrControllerDriver: Activate\n");
	m_unObjectId = unObjectId;
	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

	/* Use own render model
	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str() );
	vr::VRProperties()->SetStringProperty( m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str() );

	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "sample_controller");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "{sample}sample_controller");
	*/

	// Set render model
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ModelNumber_String, "ViveMV");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_ManufacturerName_String, "HTC");
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

	// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
	vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2);

	vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, m_role);

	// set input profile
	vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_InputProfilePath_String, "{ovr}/input/ovrcontroller_profile.json");

	// create all the input components
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/application_menu/click", &m_compA);
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/system/click", &m_compB);
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/grip/click", &m_compGrip);
	vr::VRDriverInput()->CreateBooleanComponent(m_ulPropertyContainer, "/input/joystick/click", &m_compJoystickClick);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/x", &m_compJoystickX, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/joystick/y", &m_compJoystickX, VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
	vr::VRDriverInput()->CreateScalarComponent(m_ulPropertyContainer, "/input/trigger/value", &m_compTrigger, VRScalarType_Absolute, VRScalarUnits_NormalizedOneSided);

	return VRInitError_None;
}

void COvrControllerDriver::Deactivate()
{
	DriverLog("COvrControllerDriver: Deactivate entry\n");
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	DriverLog("COvrControllerDriver: Deactivate exit\n");
}

void COvrControllerDriver::EnterStandby()
{
}

void *COvrControllerDriver::GetComponent(const char *pchComponentNameAndVersion)
{
	// override this to add a component to a driver
	return NULL;
}

void COvrControllerDriver::PowerOff()
{
}

void COvrControllerDriver::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
	if ( unResponseBufferSize >= 1 )
		pchResponseBuffer[0] = 0;
}

void COvrControllerDriver::UpdateButtons(ButtonData_t btn)
{
	double timeOffset = 0;
	vr::VRDriverInput()->UpdateBooleanComponent(m_compA, btn.buttons & 1, timeOffset);
	vr::VRDriverInput()->UpdateBooleanComponent(m_compB, btn.buttons & 2, timeOffset);
	vr::VRDriverInput()->UpdateBooleanComponent(m_compGrip, btn.buttons & 4, timeOffset);
	vr::VRDriverInput()->UpdateBooleanComponent(m_compJoystickClick, btn.buttons & 8, timeOffset);
	vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickX, btn.joyXY(0), timeOffset);
	vr::VRDriverInput()->UpdateScalarComponent(m_compJoystickY, btn.joyXY(1), timeOffset);
	vr::VRDriverInput()->UpdateScalarComponent(m_compTrigger, btn.trigger, timeOffset);
}