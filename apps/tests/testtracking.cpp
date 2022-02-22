#include <iostream>

#include "HeadMountDisplay.h"
#include "HandController.h"
#include "camera.h"

#include "tracking_math.h"
#include "camera_math.h"

#include "viewer.h"

#include "quaternion.h"

using namespace cv;

class cam_view_t {
public:
	cam_view_t() : base(0.4, 1) {}
	void init(Camera* pCam) {
		pCam->load();
		calib = pCam->getCalibration();
		base.rv = calib.rv_reverse;
		base.tv = calib.tv_reverse;
	}
	void attach(viewer& v) {
		v.viewables.push_back(&base);
		for (int i = 0; i < 3; ++i) {
			v.viewables.push_back(&rays[i]);
		}
	}
	void update(const cam_update_t& cam_data, Mat& pos) {
		Mat ray = castRay(calib.rv_reverse,
			calib.cm_reverse,
			cam_data.pixel);
		rays[cam_data.tag].origin = calib.tv_reverse;
		rays[cam_data.tag].direction = ray;
		shortestPath(calib.tv_reverse, ray, pos);
	}
private:
	Calibration calib;
	coordinate_sys base;
	arrow rays[3];
};

class device_view_t {
public:
	device_view_t() : 
		orient(0.2, 2)
	{}
	void attach(viewer& v) {
		v.viewables.push_back(&orient);
		//v.viewables.push_back(&accel);
	}
	void imu_update(const Quaternion& q, const Vec3d& lin_accel) {
		//rotate coordinate sys from quaternion
		Mat rv(3, 1, CV_64F);
		double vnorm = std::sqrt(
			q.x * q.x
			+ q.y * q.y
			+ q.z * q.z);
		double angle = std::atan2(vnorm, q.w);
		rv.at<double>(0) = angle * q.w / vnorm;
		rv.at<double>(1) = angle * q.x / vnorm;
		rv.at<double>(2) = angle * q.y / vnorm;
		orient.rv = rv;
		// TODO: set accel
		accel.direction = lin_accel;
	}
	void pos_update(const Mat& pos) {
		orient.tv = pos;
		accel.origin = pos;
	}
private:
	coordinate_sys orient;
	arrow accel;
};

int main(int argc, char* argv[]) {
	kalman_t kf[DEVICE_COUNT];

	HandController rhc, lhc;
	rhc.name = "Right Hand v2";
	lhc.name = "Left Hand Controller";
	HeadMountDisplay hmd;
	hmd.name = "Head Mount Display";

	device_view_t device_view[DEVICE_COUNT];

	std::vector<Camera*> cameraList;
	std::map<Camera*, cam_view_t*> cameraMap;
	auto devices = ps3eye::PS3EYECam::getDevices();
	for (auto& d : devices) {
		Camera* pCam = new Camera(d);
		cameraMap[pCam] = new cam_view_t;
		cameraMap[pCam]->init(pCam);
		cameraList.push_back(pCam);
	}

	viewer v("tracking view");
	coordinate_sys base;
	v.viewables.push_back(&base);
	for (auto& cam : cameraList) cameraMap[cam]->attach(v);
	for (int i = 0; i < DEVICE_COUNT; ++i) device_view[i].attach(v);
	rhc.start();
	lhc.start();
	hmd.start();
	v.start();

	while (v.isRunning()) {
		auto tbegin = std::chrono::system_clock::now();

		imu_update_t hmd_data; // TODO: hmd update

		for (int i = DEVICE_TAG_RIGHT_HAND_CONTROLLER; i < DEVICE_COUNT; ++i) {
			controller_update_t controller_data;
			HandController* pController;
			switch (i) {
			case DEVICE_TAG_RIGHT_HAND_CONTROLLER:
				pController = &rhc;
				break;
			case DEVICE_TAG_LEFT_HAND_CONTROLLER:
				pController = &lhc;
				break;
			}
			while (rhc.get_buffer(controller_data)) {
				Quaternion q(controller_data.q);
				Quaternion qz(rhc.qzero);
				Quaternion qi(rhc.qinner);

				Quaternion qn = q * qz.conjugate();

				device_view[DEVICE_TAG_RIGHT_HAND_CONTROLLER].imu_update(qn, Vec3d(controller_data.acc));
			}
		}

		for (auto& cam : cameraList) {
			cam->setTrackingStatus(
				hmd.isConnected(),
				rhc.isConnected(),
				lhc.isConnected());
			cam_update_t cam_data;
			while (cam->get_buffer(cam_data)) {
				Mat pos = kf[cam_data.tag].get().rowRange(3, 6);
				cameraMap[cam]->update(cam_data, pos);
				device_view[cam_data.tag].pos_update(pos);
			}
		}

		std::this_thread::sleep_until(tbegin + std::chrono::milliseconds(10));
	}

	rhc.stop();
	lhc.stop();
	hmd.stop();

	for (auto& cam : cameraList) {
		cam->stop();
		delete cameraMap[cam];
		delete cam;
	}
}