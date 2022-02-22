#include "camera.h"
#include "camera_math.h"

/************** Camera ****************/

Camera::Camera(ps3eye::PS3EYECam::PS3EYERef pEye) :
	width(640), height(480),
	m_pEye(pEye),
	m_autogain(false), is_calibrated(false),
	m_exposure(50), m_gain(10),
	m_threadState(false),
	m_pThread(nullptr),
	m_imBuffer(height, width, CV_8UC1),
	has_ext(false), has_int(false), has_inv(false)
{
	m_pEye->init(width, height, 60, ps3eye::PS3EYECam::EOutputFormat::Bayer);
}

Camera::~Camera() 
{
	if (m_pEye->isStreaming()) m_pEye->stop();
}

void Camera::threadFunc()
{
	cv::Mat im;
	m_threadState = true;
	while (m_threadState) {
		view(im);
		for (auto & t : trackers) {
			if (t.status) t.update(im);
		}
	}
	
}

void Camera::start(ECameraRunMode mode)
{
	m_pEye->start();
	switch (mode) {
	case ECameraRunMode::TrackingAuto:
		m_pThread = new std::thread(&Camera::threadFunc, this);
		// fall through
	case ECameraRunMode::TrackingManual:
		m_pEye->setAutogain(m_autogain);
		m_pEye->setExposure(m_exposure);
		m_pEye->setGain(m_gain);
		break;
	case ECameraRunMode::Viewing:
		m_pEye->setAutogain(true);
		break;
	}
}

void Camera::stop()
{
	if (m_pThread) {
		m_threadState = false;
		m_pThread->join();
		delete m_pThread;
		m_pThread = nullptr;
	}
	m_pEye->stop();
}

void Camera::view(cv::Mat& destination)
{
	m_pEye->getFrame(m_imBuffer.data);
	debayerPSEye(m_imBuffer, destination);
}

void Camera::man_tracker_update(EDevice dev, const cv::Mat& image)
{
	trackers[static_cast<int>(dev)].update(image);
}


bool Camera::get_tracker_ray(EDevice dev, cv::Matx31d& ray)
{
	ObjectTracker& t = trackers[static_cast<int>(dev)];
	if (t.found & t.status) {
		ray = castRay(
			m_invcalib.rv,
			m_invcalib.cm,
			t.getPoint());
		return true;
	}
	return false;
}

cv::Matx31d Camera::get_tracker_correction(const cv::Matx31d& ray, const cv::Matx31d pos)
{
	return shortestPath(m_invcalib.tv, ray, pos);
}



cv::Rect Camera::get_tracker_roi(EDevice dev)
{
	return trackers[static_cast<int>(dev)].roi;
}

cv::Mat Camera::get_tracker_mask(EDevice dev)
{
	return trackers[static_cast<int>(dev)].mask;
}

void Camera::set_tracker_bound(EDevice dev, ETrackerBound tb, cv::Vec2b bound)
{
	const int devidx = static_cast<int>(dev);
	const int boundidx = static_cast<int>(tb);
	trackers[devidx].lower_bound(boundidx) = bound(0);
	trackers[devidx].upper_bound(boundidx) = bound(1);
}

cv::Vec2b Camera::get_tracker_bound(EDevice dev, ETrackerBound tb)
{
	const int devidx = static_cast<int>(dev);
	const int boundidx = static_cast<int>(tb);
	return cv::Vec2b(
		trackers[devidx].lower_bound(boundidx),
		trackers[devidx].upper_bound(boundidx)
	);
}

void Camera::set_tracker_flags(EDevice dev, ETrackerFlag flag, bool on)
{
	const int idx = static_cast<int>(dev);
	switch (flag) {
	case ETrackerFlag::HueInvert:
		trackers[idx].invert_hue = on;
		break;
	case ETrackerFlag::Morph:
		trackers[idx].use_morph = on;
		break;
	case ETrackerFlag::Status:
		trackers[idx].status = on;
		break;
	}
}

bool Camera::get_tracker_flags(EDevice dev, ETrackerFlag flag)
{
	const int idx = static_cast<int>(dev);
	switch (flag) {
	case ETrackerFlag::HueInvert:
		return trackers[idx].invert_hue;
	case ETrackerFlag::Morph:
		return trackers[idx].use_morph;
	case ETrackerFlag::Status:
		return trackers[idx].status;
	}
	return false;
}

void Camera::set_gain(uchar gain)
{
	m_gain = gain;
	m_pEye->setGain(gain);
}

void Camera::set_exposure(uchar exposure)
{
	m_exposure = exposure;
	m_pEye->setExposure(exposure);
}

void Camera::set_autogain(bool value)
{
	m_autogain = value;
	m_pEye->setAutogain(value);
}

void Camera::set_intrinsics(const cv::Matx33d& cm, cv::Matx<double, 5, 1> dc)
{
	m_calib.dc = dc;
	m_calib.cm = cm;
	has_int = true;
	if (!is_calibrated && has_ext) {
		is_calibrated = true;
		get_reverse_calibration();
	}
}

void Camera::set_extrinsics(const cv::Matx31d& rv, const cv::Matx31d& tv)
{
	m_calib.tv = tv;
	m_calib.rv = rv;
	has_ext = true;
	if (!is_calibrated && has_int) {
		is_calibrated = true;
		get_reverse_calibration();
	}
}

Calibration_t Camera::get_reverse_calibration()
{
	if (!has_inv) {
		cv::Matx33d rmat;
		cv::Rodrigues(m_calib.rv, rmat);
		cv::Rodrigues(rmat.t(), m_invcalib.rv);
		m_invcalib.tv = -rmat.t() * m_calib.tv;
		m_invcalib.cm = m_calib.cm.inv();
		m_invcalib.dc = m_calib.dc;
		has_inv = true;
	}

	return m_invcalib;
}

void Camera::write(cv::FileStorage& fs) const
{
	fs <<"{"
		<< "Rotation Vector" << m_calib.rv
		<< "Translation Vector" << m_calib.tv
		<< "Camera Matrix" << m_calib.cm
		<< "Distortion Coefficients" << m_calib.dc
		<< "HMD" << trackers[0]
		<< "RHC" << trackers[1]
		<< "LHC" << trackers[2]
		<< "Exposure" << m_exposure
		<< "Gain" << m_gain
		<< "Auto" << m_autogain
		<< "}";
}

void Camera::read(const cv::FileNode& node)
{
	node["Rotation Vector"] >> m_calib.rv;
	node["Translation Vector"] >> m_calib.tv;
	node["Camera Matrix"] >> m_calib.cm;
	node["Distortion Coefficients"] >> m_calib.dc;
	node["HMD"] >> trackers[0];
	node["RHC"] >> trackers[1];
	node["LHC"] >> trackers[2];
	node["Exposure"] >> m_exposure;
	node["Gain"] >> m_gain;
	node["Auto"] >> m_autogain;
	get_reverse_calibration();
	is_calibrated = true;
}

std::string Camera::file_id()
{
	char fid[16];
	m_pEye->getUSBPortPath(fid, sizeof(fid));
	return std::string(fid);
}

/************** Function definitions ****************/

void debayerPSEye(cv::Mat& bayer, cv::Mat& bgr)
{
	cvtColor(bayer, bgr, cv::COLOR_BayerGB2BGR);
}

