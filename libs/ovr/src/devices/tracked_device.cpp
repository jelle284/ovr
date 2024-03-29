#include "tracked_device.h"
#include "mahony.h"
#include "madgwick.h"
#include "camera_math.h"

TrackerBase::TrackerBase(EDevice tag) :
    m_tag(tag),
	m_pollRate(10),
	m_running(false),
	led_R(255), led_G(255), led_B(255), led_tmr(1),
	m_connectionStatus(false)
{
	m_connectionStatus = false;
	
	m_kf.makeAB(0.001 * m_pollRate);
	m_kf.setNoise(0.6, 0.4);
}


TrackerBase::~TrackerBase()
{
	if (m_running) {
		m_running = false;
	}
	m_connectionStatus = false;
}

void TrackerBase::imu_update(float* read_buf)
{
    const std::lock_guard<std::mutex> lock(m_mtx);
	// read from buf
	for (int i = 0; i < 3; ++i) {
		m_IMUData.accel(i) = read_buf[i + REG_ACC];
		m_IMUData.gyro(i) = read_buf[i + REG_GYRO];
		m_IMUData.mag(i) = read_buf[i + REG_MAG];
	}
	fuse_imu();
}

void TrackerBase::fuse_imu()
{
	// imu fusion
	auto dq = mahonyQuaternionUpdate(m_IMUData, m_q);
	m_q = m_q + dq.mul(m_pollRate * 0.001f);
	m_q = m_q.normalized();
	// offset
	m_Pose.q = m_qzero.conjugate() * m_q;
	// update acceleration
	Quaternionf qacc = m_q * Quaternionf(0.0, m_IMUData.accel(0), m_IMUData.accel(1), m_IMUData.accel(2)) * m_q.conjugate();
	cv::Matx31d acc(qacc.val[1], qacc.val[2] - 9.81, qacc.val[3]);
	m_kf.setAcc(acc);
	m_kf.predict();
	// set flag
	has_data |= EDataFlags::IMU;
}

IMUData_t TrackerBase::get_imu_data()
{
    const std::lock_guard<std::mutex> lock(m_mtx);
	has_data &= ~EDataFlags::IMU;
    return m_IMUData;
}

Pose_t TrackerBase::get_pose_data()
{
	const std::lock_guard<std::mutex> lock(m_mtx);
	m_Pose.pos = cv::Matx31f(m_kf.get_pos());
	m_Pose.vel = cv::Matx31f(m_kf.get_vel());
	has_data &= ~EDataFlags::Pose;
	return m_Pose;
}

void TrackerBase::update_from_ext(std::vector<ICamera*> cameras)
{
	const std::lock_guard<std::mutex> lock(m_mtx);
	using namespace cv;
	Matx31d step(0, 0, 0);
	for (auto& c : cameras) {
		Matx31d ray;
		if (c->get_tracker_ray(m_tag, ray)) {
			auto dir = c->get_tracker_correction(ray, m_Pose.pos);
			step += dir/(double)cameras.size();
		}
	}
	auto next_pos = m_kf.get_pos() + step;
	m_kf.correct(next_pos);

	has_data |= EDataFlags::Pose;
	has_data &= ~EDataFlags::IMU;
}

void TrackerBase::update_from_int(const cv::Point3d& head)
{
	// rotation only position estimation
}

void TrackerBase::align()
{
	const std::lock_guard<std::mutex> lock(m_mtx);
	m_qzero = m_q;
	m_accRef = m_IMUData.accel;
	m_magRef = m_IMUData.mag;
}

void TrackerBase::write(cv::FileStorage& fs) const
{
	fs << "{"
		<< "LED Red" << led_R
		<< "LED Green" << led_G
		<< "LED Blue" << led_B
		<< "LED Timer" << led_tmr
		<< "Align W" << m_qzero.val[0]
		<< "Align X" << m_qzero.val[1]
		<< "Align Y" << m_qzero.val[2]
		<< "Align Z" << m_qzero.val[3]
		<< "Mag Ref" << m_magRef
		<< "Acc Ref" << m_accRef;
	udef_write(fs);
	fs << "}";

}

void TrackerBase::read(const cv::FileNode& node)
{
	node["LED Red"] >> led_R;
	node["LED Green"] >> led_G;
	node["LED Blue"] >> led_B;
	node["LED Timer"] >> led_tmr;
	node["Align W"] >> m_qzero.val[0];
	node["Align X"] >> m_qzero.val[1];
	node["Align Y"] >> m_qzero.val[2];
	node["Align Z"] >> m_qzero.val[3];
	node["Mag Ref"] >> m_magRef;
	node["Acc Ref"] >> m_accRef;
	udef_read(node);
	m_q = m_qzero;
}



std::string TrackerBase::get_name() {
	switch (m_tag)
	{
	case EDevice::Hmd:
		return std::string("Head Mount Display");
	case EDevice::LeftHandController:
		return std::string("Left Hand Controller");
	case EDevice::RightHandController:
		return std::string("Right Hand Controller");
	default:
		return std::string("Unknown device");
	}
}