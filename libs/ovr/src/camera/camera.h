#ifndef CAMERA_H
#define CAMERA_H

#include "opencv2/opencv.hpp"
#include "ps3eye.h"

#include "ovr.h"
#include "object_tracker.h"

class TrackerBase;

class Camera : public ICamera {
public:
	Camera(ps3eye::PS3EYECam::PS3EYERef pEye);
	~Camera();
	void registerDevice(TrackerBase* pDevice);

	// interface implemention
	virtual void start(ECameraRunMode mode) override;
	virtual void stop() override;

	virtual void view(cv::Mat& destination) override;
	virtual void man_tracker_update(EDevice dev, const cv::Mat& image) override;

	virtual bool get_tracker_ray(EDevice dev, cv::Matx31d& ray) override;
	virtual cv::Matx31d get_tracker_correction(const cv::Matx31d& ray, const cv::Matx31d pos) override;
	virtual cv::Rect get_tracker_roi(EDevice dev) override;
	virtual cv::Mat get_tracker_mask(EDevice dev) override;

	virtual void set_tracker_bound(EDevice dev, ETrackerBound tb, cv::Vec2b bound) override;
	virtual cv::Vec2b get_tracker_bound(EDevice dev, ETrackerBound tb) override;

	virtual void set_tracker_flags(EDevice dev, ETrackerFlag flag, bool on) override;
	virtual bool get_tracker_flags(EDevice dev, ETrackerFlag flag) override;

	virtual void set_gain(uchar gain) override;
	virtual uchar get_gain() { return m_gain; }

	virtual void set_exposure(uchar exposure) override;
	virtual uchar get_exposure() { return m_exposure; }

	virtual void set_autogain(bool value) override;
	virtual bool get_autogain() { return m_autogain; }

	virtual void set_intrinsics(const cv::Matx33d& cm, cv::Matx<double, 5, 1> dc)  override;
	virtual void set_extrinsics(const cv::Matx31d& rv, const cv::Matx31d& tv) override;

	virtual Calibration_t get_reverse_calibration() override;
	virtual Calibration_t get_calibration() override { return m_calib; }

	void write(cv::FileStorage& fs) const override;
	void read(const cv::FileNode& node) override;
	virtual std::string file_id() override;

private:
	std::array<TrackerBase*, 3> devices;
	const unsigned int width, height;
	cv::Mat m_imBuffer;
	std::thread* m_pThread;
	bool m_threadState;
	ps3eye::PS3EYECam::PS3EYERef m_pEye;

	uchar m_gain, m_exposure;
	bool m_autogain, is_calibrated;
	Calibration_t m_calib, m_invcalib;
	std::array<ObjectTracker, 3> trackers;

	void threadFunc();

	bool has_inv, has_int, has_ext;
}; 

void debayerPSEye(cv::Mat& bayer, cv::Mat& bgr);

#endif