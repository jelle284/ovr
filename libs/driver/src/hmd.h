#ifndef HMD_H
#define HMD_H

#include "common.h"

//-----------------------------------------------------------------------------
// Purpose: Head Mount Display
//-----------------------------------------------------------------------------
class COvrHMDDriver : 
	public DriverTrackedDevice,
	public vr::IVRDisplayComponent
{
public:
	COvrHMDDriver();

	virtual ~COvrHMDDriver();

	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId);

	virtual void Deactivate();

	virtual void EnterStandby();

	void* GetComponent(const char* pchComponentNameAndVersion);

	virtual void PowerOff();

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize);

	virtual void GetWindowBounds(int32_t* pnX, int32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight);

	virtual bool IsDisplayOnDesktop();

	virtual bool IsDisplayRealDisplay();

	virtual void GetRecommendedRenderTargetSize(uint32_t* pnWidth, uint32_t* pnHeight);

	virtual void GetEyeOutputViewport(EVREye eEye, uint32_t* pnX, uint32_t* pnY, uint32_t* pnWidth, uint32_t* pnHeight);

	virtual void GetProjectionRaw(EVREye eEye, float* pfLeft, float* pfRight, float* pfTop, float* pfBottom);

	virtual DistortionCoordinates_t ComputeDistortion(EVREye eEye, float fU, float fV);

private:
	int32_t m_nWindowX;
	int32_t m_nWindowY;
	int32_t m_nWindowWidth;
	int32_t m_nWindowHeight;
	int32_t m_nRenderWidth;
	int32_t m_nRenderHeight;
	float m_flSecondsFromVsyncToPhotons;
	float m_flDisplayFrequency;
	float m_flIPD;
	float m_fK1, m_fK2;
};

#endif // !HMD_H