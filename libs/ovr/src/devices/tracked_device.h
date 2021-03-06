#ifndef TRACKEDOBJECT_H
#define TRACKEDOBJECT_H

#include "ovr.h"
#include "tracking_math.h"

#include <mutex>
#include <chrono>

/* Data transfer protocol */

// Message bytes
#define MSG_FC      0
#define MSG_ADDR    1
#define MSG_LEN     2
#define MSG_DATA    4

// Function codes
#define FC_ZERO     0
#define FC_READ     1
#define FC_WRITE    2
#define FC_ERROR    3
#define FC_COMMIT   5

// Read only register map
#define REG_ACC     0x00
#define REG_GYRO    0x03
#define REG_MAG     0x06
#define REG_FLAGS   0x09
#define REG_JOY     0x0A
#define REG_TRGBTN  0x0B
#define REG_RAWA    0x0C
#define REG_RAWG    0x0E
#define REG_RAWM    0x10

// Read + Write register map
#define REG_LED     0x16
#define REG_YAWOFS  0x17
#define REG_NAME    0x18
#define REG_MBIAS   0x20
#define REG_GBIAS   0x22

#define REG_JMIN    0x28
#define REG_JMAX    0x29
#define REG_JMID    0x2A
#define REG_TRNG    0x2B
#define REG_BTN1    0x2C
#define REG_BTN2    0x2D
#define REG_BTN3    0x2E
#define REG_BTN4    0x2F
#define REG_SSID    0x30
#define REG_PASS    0x38

/* Base class for tracked devices */
class TrackerBase : public IDevice
{
private:
	std::mutex m_mtx;
	IMUData_t m_IMUData;
	Pose_t m_Pose;
	Quaternionf m_qzero, m_q;
	cv::Matx31f m_accRef, m_magRef;
	KalmanFilter3 m_kf;
	
protected:
	EDevice m_tag;
	bool m_connectionStatus;
	bool m_threadState;
	int m_pollRate;
	void imu_update(float* read_buf);

public:
	TrackerBase(EDevice tag);
	~TrackerBase();
	
	/* device interface */
	virtual EDevice getTag() override { return m_tag; }
	virtual bool isConnected() override { return m_connectionStatus; }
	virtual bool isRunning() override { return m_threadState; }
	virtual void setPollRate(int ms) override { m_pollRate = ms; }
	virtual std::string get_name() override;

	virtual void update_from_ext(std::vector<ICamera*> cameras) override;
	virtual void update_from_int(const cv::Point3d& head) override;

	virtual IMUData_t get_imu_data() override;
	virtual Pose_t get_pose_data() override;

	virtual void align() override;
	virtual void write(cv::FileStorage& fs) const override;
	virtual void read(const cv::FileNode& node) override;
};

#endif