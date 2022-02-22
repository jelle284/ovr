#include "sensor_fusion.h"
#include "camera_math.h"
#include "tracking_math.h"

SensorFusion::SensorFusion()
{
}

SensorFusion::~SensorFusion()
{
}

void SensorFusion::camUpdate(std::vector<Camera*> cameras, EDeviceTag tag)
{
    auto filt_state = get_state();
    cv::Matx31d pos(filt_state(0), filt_state(1), filt_state(2));
    cv::Matx31d step(0, 0, 0);

    for (auto & c : cameras) {
        if (c->arrTracker[tag].found) {
            auto pixel = c->arrTracker[tag].getPoint();
            Calibration& calib = c->calibration;
            auto ray = castRay(calib.rv_reverse, calib.cm_reverse, pixel);
            auto dir = shortestPath(calib.tv_reverse, ray, pos);
            step += dir;
        }
    }
    if (cv::norm(step) > 0.1) {
        step = 0.1 * step / cv::norm(step);
    }
    correct(pos + step);
}

void SensorFusion::imuUpdate(imu_update_t imu_data)
{
    double deltat = std::chrono::duration<double>(imu_data.timestamp - t_imu_last).count();
    t_imu_last = imu_data.timestamp;
    if (deltat > 1.0) return;
    auto dq = mahonyQuaternionUpdate(
        imu_data.acc[0], imu_data.acc[1], imu_data.acc[2],
        imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2],
        imu_data.mag[0], imu_data.mag[1], imu_data.mag[2],
        q.val[0], q.val[1], q.val[2], q.val[3]);
    /*
    auto dq = magdwickQuaternionUpdate(
        q,
        cv::Matx31d(imu_data.acc),
        cv::Matx31d(imu_data.gyro),
        cv::Matx31d(imu_data.mag));
        */
    q = q.integrate(dq, deltat);

    // calculate acceleration in world
    Quaternion qacc = q * Quaternion(0.0, imu_data.acc[0], imu_data.acc[1], imu_data.acc[2]) * q.conjugate();
    cv::Matx31d acc(qacc.val[1], qacc.val[2] - 9.81, qacc.val[3]);
    setAcc(acc);
    // predict state
    makeAB(deltat);
    predict();
}