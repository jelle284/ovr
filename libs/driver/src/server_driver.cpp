#include "server_driver.h"



EVRInitError COvrServerDriver::Init( vr::IVRDriverContext *pDriverContext )
{
	DriverLog("COvrServerDriver: Init\n");
	VR_INIT_SERVER_DRIVER_CONTEXT( pDriverContext );
	InitDriverLog( vr::VRDriverLog() );

	DeviceList = getDevices();
	CameraList = getCameras();

	m_pHMD = new COvrHMDDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded("ovr-hmd", vr::TrackedDeviceClass_HMD, m_pHMD);
	
	m_pRHController = new COvrControllerDriver(TrackedControllerRole_RightHand);
	vr::VRServerDriverHost()->TrackedDeviceAdded("ovr-rhc", vr::TrackedDeviceClass_Controller, m_pRHController);

	m_pLHController = new COvrControllerDriver(TrackedControllerRole_LeftHand);
	vr::VRServerDriverHost()->TrackedDeviceAdded("ovr-lhc", vr::TrackedDeviceClass_Controller, m_pLHController);


	for (auto c : CameraList) {
		auto pBaseStation = new COvrBaseStation();
			vr::VRServerDriverHost()->TrackedDeviceAdded(
				c->file_id().c_str(), 
				vr::TrackedDeviceClass_TrackingReference,
				pBaseStation
			);
	}

	// open calibration file
	auto propHandle = pDriverContext->GetDriverHandle();
	auto ucp = vr::VRProperties()->GetStringProperty(propHandle, ETrackedDeviceProperty::Prop_UserConfigPath_String);
	
	cv::FileStorage fs(ucp + "\\vr_calib.json", cv::FileStorage::READ);
	if (fs.isOpened()) {
		DriverLog("Using calibration file %s\\vr_calib.json", ucp.c_str());
		for (auto c : CameraList) {
			fs[c->file_id()] >> c;
			auto cal = c->get_calibration();
			DriverLog("Camera %s calibrated.\n",
				c->file_id().c_str());
		}
		for (auto d : DeviceList) {
			fs[d->get_name()] >> d;
			DriverLog("Device %s calibrated.",
				d->get_name().c_str());
		}
	}
	else {
		DriverLog("Failed to get calibration from: %s.\nvr_calib.json not found!", ucp.c_str());
	}


	m_pTrackingThread = new std::thread(&COvrServerDriver::threadFunc, this);

	return VRInitError_None;
}

void COvrServerDriver::Cleanup() 
{
	DriverLog("COvrServerDriver: Cleanup\n");
	m_TrackingThreadState = false;
	m_pTrackingThread->join();
	delete m_pTrackingThread;
	m_pTrackingThread = NULL;
	delete m_pHMD;
	m_pHMD = NULL;
	delete m_pRHController;
	m_pRHController = NULL;
	delete m_pLHController;
	m_pLHController = NULL;
	for (auto& p : m_pBaseStations) {
		delete p;
		p = NULL;
	}
	m_pBaseStations.clear();
	DriverLog("COvrServerDriver: Cleaning up Driver Log\n");
	CleanupDriverLog();
}


void COvrServerDriver::RunFrame()
{
		m_pHMD->RunFrame();
		m_pRHController->RunFrame();
		m_pLHController->RunFrame();
}

void COvrServerDriver::threadFunc()
{
	DriverLog("COvrServerDriver: Thread entry\n");
	const int kLoopPeriodms = 10;

	std::map<EDevice, DriverTrackedDevice*> DriverDevices{
		{ EDevice::Hmd, m_pHMD },
		{ EDevice::LeftHandController, m_pLHController },
		{ EDevice::RightHandController, m_pRHController }
	};

	for (auto d : DeviceList) d->start();
	for (auto c : CameraList) c->start(ECameraRunMode::TrackingAuto);

	m_TrackingThreadState = true;
	while (m_TrackingThreadState) {
		auto loopDeadline = std::chrono::system_clock::now() + std::chrono::milliseconds(kLoopPeriodms);
		// update devices
		for (auto d : DeviceList) {
			EDevice tag = d->getTag();
			DriverTrackedDevice* pDev = DriverDevices[tag];

			for (auto c : CameraList) c->set_tracker_flags(tag, ETrackerFlag::Status, d->isConnected());

			if (d->has_data & EDataFlags::IMU) {
				d->update_from_ext(CameraList);
			}
			

			if (d->has_data & EDataFlags::Pose) {
				auto pose = d->get_pose_data();
				DriverPose_t DriverPose = pDev->GetPose();
				DriverPose.deviceIsConnected = d->isConnected();
				DriverPose.poseIsValid = true;
				DriverPose.result = TrackingResult_Running_OK;
				for (int i = 0; i < 3; ++i) {
					DriverPose.vecPosition[i] = (double)pose.pos(i);
					DriverPose.vecVelocity[i] = (double)pose.vel(i);
				}
				DriverPose.qRotation.w = pose.q.val[0];
				DriverPose.qRotation.x = pose.q.val[1];
				DriverPose.qRotation.y = pose.q.val[2];
				DriverPose.qRotation.z = pose.q.val[3];
				pDev->UpdatePose(DriverPose);
			}

			if (d->has_data & EDataFlags::Button) {
				auto btn = d->get_button_data();
				pDev->UpdateButtons(btn);
			}

		}
		//
		std::this_thread::sleep_until(loopDeadline);
	}

	for (auto& d : DeviceList) d->stop();
	for (auto& c : CameraList) c->stop();
	DriverLog("COvrServerDriver: Thread exit\n");
}