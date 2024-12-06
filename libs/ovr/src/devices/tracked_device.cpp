#include "tracked_device.h"
#include "camera_math.h"
#include "camera.h"

double timer_tick(std::chrono::system_clock::time_point& t0, const double upper_lim = 1.0) {
	std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
	std::chrono::duration<double> elapsed = t1 - t0;
	t0 = t1;
	if (elapsed.count() > upper_lim) return upper_lim;
	if (elapsed.count() <= 0.0) return 0.001;
	return elapsed.count();
}

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
	m_t0 = std::chrono::system_clock::now();
}


TrackerBase::~TrackerBase()
{
	if (m_running) {
		m_running = false;
	}
	m_connectionStatus = false;
}

void TrackerBase::camera_update(Camera* cam)
{
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
	using namespace cv;
	// get timing
	double elapsed = timer_tick(m_t0);
	const float Kp = 1.0f;
	// prep quaternion
	if (isnan(m_q.val[0])) m_q = Quaternionf(1.0f, 0.0f, 0.0f, 0.0f);
	Matx33f rm = m_q.toRotationMatrix(); // converts sensor frame to world frame
	// accelerometer correction
	Matx31f a_norm = m_IMUData.accel / norm(m_IMUData.accel);
	Matx31f a_ref = rm.t() * Matx31f(0.0f,1.0f,0.0f);
	Matx31f a_corr = Kp * cross(a_norm, a_ref);
	Quaternionf qa(a_corr);

	// magnetometer correction
	Matx31f m_norm = m_IMUData.mag / norm(m_IMUData.mag);
	Matx31f m_world = rm * m_norm;
	Matx21f compass(m_world(0), m_world(2));
	float incline = m_world(1);
	Matx31f m_ref = rm.t() * Matx31f(0.0f, incline, -norm(compass));
	Matx31f m_corr = Kp * cross(m_norm, m_ref);
	Quaternionf qm(m_corr);

	// quaternion integration
	Quaternionf qw(m_IMUData.gyro);
	Quaternionf dq = 0.5f * m_q * (qw+qa+qm);
	m_q = m_q + dq * elapsed;
	m_q = m_q.normalized();
	// offset
	m_Pose.q = m_qzero.conjugate() * m_q;

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
	m_kf.predict();
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