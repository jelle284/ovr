#include "hmd.h"
#include "quaternion.h"

// keys for use with the settings API
static const char* const k_pch_Section = "driver_ovr";
static const char* const k_pch_WindowX_Int32 = "windowX";
static const char* const k_pch_WindowY_Int32 = "windowY";
static const char* const k_pch_WindowWidth_Int32 = "windowWidth";
static const char* const k_pch_WindowHeight_Int32 = "windowHeight";
static const char* const k_pch_RenderWidth_Int32 = "renderWidth";
static const char* const k_pch_RenderHeight_Int32 = "renderHeight";
static const char* const k_pch_SecondsFromVsyncToPhotons_Float = "secondsFromVsyncToPhotons";
static const char* const k_pch_DisplayFrequency_Float = "displayFrequency";

COvrHMDDriver::COvrHMDDriver()
{
	DriverLog("COvrHMDDriver: Constructor\n");
	m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;

	m_flIPD = vr::VRSettings()->GetFloat( k_pch_SteamVR_Section, k_pch_SteamVR_IPD_Float );
	
	m_nWindowX = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_WindowX_Int32 );
	m_nWindowY = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_WindowY_Int32 );
	m_nWindowWidth = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_WindowWidth_Int32 );
	m_nWindowHeight = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_WindowHeight_Int32 );
	m_nRenderWidth = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_RenderWidth_Int32 );
	m_nRenderHeight = vr::VRSettings()->GetInt32( k_pch_Section, k_pch_RenderHeight_Int32 );
	m_flSecondsFromVsyncToPhotons = vr::VRSettings()->GetFloat( k_pch_Section, k_pch_SecondsFromVsyncToPhotons_Float );
	m_flDisplayFrequency = vr::VRSettings()->GetFloat( k_pch_Section, k_pch_DisplayFrequency_Float );

	m_fK1 = vr::VRSettings()->GetFloat(k_pch_Section, "K1");
	m_fK2 = vr::VRSettings()->GetFloat(k_pch_Section, "K2");
}
	
COvrHMDDriver::~COvrHMDDriver()
{
}

EVRInitError COvrHMDDriver::Activate(vr::TrackedDeviceIndex_t unObjectId) 
{
	DriverLog("COvrHMDDriver: Activate\n");
	m_unObjectId = unObjectId;
	m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer( m_unObjectId );

	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserIpdMeters_Float, m_flIPD );
	DriverLog("Setting IDP to %f", m_flIPD);
	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_UserHeadToEyeDepthMeters_Float, 0.f );
	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_DisplayFrequency_Float, m_flDisplayFrequency );
	vr::VRProperties()->SetFloatProperty( m_ulPropertyContainer, Prop_SecondsFromVsyncToPhotons_Float, m_flSecondsFromVsyncToPhotons );

	// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
	vr::VRProperties()->SetUint64Property( m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 2 );

	// avoid "not fullscreen" warnings from vrmonitor
	vr::VRProperties()->SetBoolProperty( m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false );

	return VRInitError_None;
}

void COvrHMDDriver::Deactivate() 
{
	DriverLog("COvrHMDDriver: Deactivate entry\n");
	m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	DriverLog("COvrHMDDriver: Deactivate exit\n");
}

void COvrHMDDriver::EnterStandby()
{
}

void *COvrHMDDriver::GetComponent(const char *pchComponentNameAndVersion)
{
	if ( !_stricmp( pchComponentNameAndVersion, vr::IVRDisplayComponent_Version ) )
	{
		return (vr::IVRDisplayComponent*)this;
	}

	// override this to add a component to a driver
	return NULL;
}

void COvrHMDDriver::PowerOff() 
{
}

void COvrHMDDriver::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) 
{
	if(unResponseBufferSize >= 1)
		pchResponseBuffer[0] = 0;
}

void COvrHMDDriver::GetWindowBounds(int32_t *pnX, int32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight) 
{
	// auto monitor detection

	// fallback to settings
	*pnX = m_nWindowX;
	*pnY = m_nWindowY;
	*pnWidth = m_nWindowWidth;
	*pnHeight = m_nWindowHeight;
}

bool COvrHMDDriver::IsDisplayOnDesktop() 
{
	return true;
}

bool COvrHMDDriver::IsDisplayRealDisplay() 
{
	return true;
}

void COvrHMDDriver::GetRecommendedRenderTargetSize( uint32_t *pnWidth, uint32_t *pnHeight ) 
{
	*pnWidth = m_nRenderWidth;
	*pnHeight = m_nRenderHeight;
}

void COvrHMDDriver::GetEyeOutputViewport(EVREye eEye, uint32_t *pnX, uint32_t *pnY, uint32_t *pnWidth, uint32_t *pnHeight) 
{
	*pnY = 0;
	*pnWidth = m_nWindowWidth / 2;
	*pnHeight = m_nWindowHeight;

	if ( eEye == Eye_Left )
	{
		*pnX = 0;
	}
	else
	{
		*pnX = m_nWindowWidth / 2;
	}
}

void COvrHMDDriver::GetProjectionRaw(EVREye eEye, float *pfLeft, float *pfRight, float *pfTop, float *pfBottom) 
{
	*pfLeft = -1.0;
	*pfRight = 1.0;
	*pfTop = -1.0;
	*pfBottom = 1.0;	
}

DistortionCoordinates_t COvrHMDDriver::ComputeDistortion(EVREye eEye, float fU, float fV) 
{
	DistortionCoordinates_t coordinates;
	float rr = (fU - 0.5f)*(fU - 0.5f) + (fV - 0.5f)*(fV - 0.5f);
	float coeff = 1 + m_fK1 * rr + m_fK2 * rr * rr;
	float fdU = 0.5f + (fU - 0.5f) * coeff;
	float fdV = 0.5f + (fV - 0.5f) * coeff;
	// TODO: tangential distortion?
	coordinates.rfBlue[0] = fdU;
	coordinates.rfBlue[1] = fdV;
	coordinates.rfGreen[0] = fdU;
	coordinates.rfGreen[1] = fdV;
	coordinates.rfRed[0] = fdU;
	coordinates.rfRed[1] = fdV;
	return coordinates;
}