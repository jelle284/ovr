#ifndef OVR_H
#define OVR_H

#include "opencv2/opencv.hpp"
#include "quaternion.h"
#include <vector>

#define OVR_DLL_EXPORT __declspec(dllexport)

enum class EDevice {
	Hmd = 0,
	LeftHandController,
	RightHandController
};

/*
* 
							CAMERA INTERFACE
							----------------
*/
enum class ETrackerBound { 
	Hue,
	Sat,
	Value 
};

enum class ETrackerFlag {
	Status,
	Morph,
	HueInvert
};

enum class ECameraRunMode {
	TrackingAuto,
	TrackingManual,
	Viewing
};

struct Calibration_t {
	cv::Matx31d rv, tv;
	cv::Matx33d cm;
	cv::Matx<double, 5, 1> dc;
};

class ICamera {
public:
	virtual void start(ECameraRunMode mode) = 0;
	virtual void stop() = 0;

	virtual void view(cv::Mat& destination) = 0;
	virtual void man_tracker_update(EDevice dev, const cv::Mat& image) = 0;

	virtual bool get_tracker_ray(EDevice dev, cv::Matx31d& ray) = 0;
	virtual cv::Matx31d get_tracker_correction(const cv::Matx31d& ray, const cv::Matx31d pos) = 0;
	virtual cv::Rect get_tracker_roi(EDevice dev) = 0;
	virtual cv::Mat get_tracker_mask(EDevice dev) = 0;

	virtual void set_tracker_bound(EDevice dev, ETrackerBound tb, cv::Vec2b bound) = 0;
	virtual cv::Vec2b get_tracker_bound(EDevice dev, ETrackerBound tb) = 0;

	virtual void set_tracker_flags(EDevice dev, ETrackerFlag flag, bool on) = 0;
	virtual bool get_tracker_flags(EDevice dev, ETrackerFlag flag) = 0;

	virtual void set_gain(uchar gain) = 0;
	virtual uchar get_gain() = 0;

	virtual void set_exposure(uchar exposure) = 0;
	virtual uchar get_exposure() = 0;

	virtual void set_autogain(bool value) = 0;
	virtual bool get_autogain() = 0;

	virtual void set_intrinsics(const cv::Matx33d& cm, cv::Matx<double, 5, 1> dc) = 0;
	virtual void set_extrinsics(const cv::Matx31d& rv, const cv::Matx31d& tv) = 0;

	virtual Calibration_t get_reverse_calibration() = 0;
	virtual Calibration_t get_calibration() = 0;

	virtual void write(cv::FileStorage& fs) const = 0;
	virtual void read(const cv::FileNode& node) = 0;
	virtual std::string file_id() = 0;
};

static void write(cv::FileStorage& fs, const std::string&, const ICamera* camera) {
	camera->write(fs);
}

static void read(const cv::FileNode& node, ICamera* camera, const ICamera*) {
	if (!node.empty()) {
		camera->read(node);
	}
}
typedef std::vector<ICamera*> CameraList_t;

OVR_DLL_EXPORT CameraList_t getCameras();

/*
* 
							TRACKED DEVICE INTERFACE
							------------------------
*/
struct IMUData_t {
	cv::Matx31f accel, gyro, mag;
};

struct ADCData_t {
	int16_t
		jx,
		jy,
		trig,
		btn;
};

struct ButtonData_t {
	cv::Matx21f joyXY;
	double trigger;
	uint buttons;
};

struct Pose_t {
	Quaternionf q;
	cv::Matx31f vel, pos;
};

namespace EDataFlags {
	enum Flags {
		IMU = 0x01,
		Button = 0x02,
		Pose = 0x04,
	};
}

class IDevice {
public:
	uchar has_data;

	virtual void start() = 0;
	virtual void stop() = 0;

	virtual void update_from_ext(std::vector<ICamera*> cameras) = 0;
	virtual void update_from_int(const cv::Point3d& head) = 0;

	virtual IMUData_t get_imu_data() { return IMUData_t(); };
	virtual ButtonData_t get_button_data() { return ButtonData_t(); };
	virtual ADCData_t get_adc_data() { return ADCData_t(); };
	virtual Pose_t get_pose_data() { return Pose_t(); };
	virtual std::string get_name() = 0;
	virtual EDevice getTag() = 0;
	virtual bool isConnected() = 0;
	virtual bool isRunning() = 0;
	virtual void setPollRate(int ms) = 0;
	virtual void align() = 0;
	virtual void write(cv::FileStorage& fs) const = 0;
	virtual void read(const cv::FileNode& node) = 0;
};

static void write(cv::FileStorage& fs, const std::string&, const IDevice* dev) {
	dev->write(fs);
}

static void read(const cv::FileNode& node, IDevice* dev, const IDevice*) {
	if (!node.empty()) {
		dev->read(node);
	}
}
typedef std::array<IDevice*, 3> DeviceList_t;
OVR_DLL_EXPORT DeviceList_t getDevices();

#endif