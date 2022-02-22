#include "mahony.h"

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

Quaternionf mahonyQuaternionUpdate(IMUData_t imu, Quaternionf q)
{
    float
        q0 = q.val[0],
        q1 = q.val[1],
        q2 = q.val[2],
        q3 = q.val[3],
        ax = imu.accel(0),
        ay = imu.accel(1),
        az = imu.accel(2),
        gx = imu.gyro(0),
        gy = imu.gyro(1),
        gz = imu.gyro(2),
        mx = imu.mag(0),
        my = imu.mag(1),
        mz = imu.mag(2);


    float Kp = 2.0f;
    // Local variables
    float recipNorm;
    float hx, hz, by, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
    float dq0, dq1, dq2, dq3;

    // Define auxiliary variables
    float x0 = 2 * q0;
    float x1 = q3 * x0;
    float x2 = 2 * q2;
    float x3 = q1 * x2;
    float x4 = x1 + x3;
    float x5 = -2 * q1 * q1;
    float x6 = 1 - 2 * q3 * q3;
    float x7 = x5 + x6;
    float x8 = q1 * x0;
    float x9 = q3 * x2;
    float x10 = -x8 + x9;
    float x11 = q2 * x0;
    float x12 = 2 * q1 * q3;
    float x13 = -2 * q2 * q2;
    float x14 = -x11 + x12;
    float x15 = x8 + x9;
    float x16 = x13 + x5 + 1;

    // Zero checks
    bool magCheck = ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f));
    bool accelCheck = ((ax != 0.0f) && (ay != 0.0f) && (az != 0.0f));
    if (magCheck && accelCheck) {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        // Reference direction of earth's magenitic field
        hx = mx * (x13 + x6) + my * (-x1 + x3) + mz * (x11 + x12);
        hz = mx * x14 + my * x15 + mz * x16;
        by = mx * x4 + my * x7 + mz * x10;
        bz = sqrt(hx * hx + hz * hz);
        // Estimated accelerometer measurement
        vx = x4;
        vy = x7;
        vz = x10;
        // Estimated magnetometer measurement
        wx = by * x4 + bz * x14;
        wy = by * x7 + bz * x15;
        wz = by * x10 + bz * x16;
        // Calculate error
        ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        // Apply feedback
        gx += Kp * ex;
        gy += Kp * ey;
        gz += Kp * ez;
        // Quaternion rate of change
        dq0 = -0.5f * (gx * q1 + gy * q2 + gz * q3);
        dq1 = 0.5f * (gx * q0 - gy * q3 + gz * q2);
        dq2 = 0.5f * (gx * q3 + gy * q0 - gz * q1);
        dq3 = 0.5f * (-gx * q2 + gy * q1 + gz * q0);
        return Quaternionf(dq0, dq1, dq2, dq3);
    }
    return Quaternionf(0.0, 0.0, 0.0, 0.0);
}

Quaternionf deadReckon(cv::Matx31f gyro, Quaternionf q)
{
    float dq0, dq1, dq2, dq3;
    dq0 = -0.5f * (gyro(0) * q(1) + gyro(1) * q(2) + gyro(2) * q(3));
    dq1 = 0.5f * (gyro(0) * q(0) - gyro(1) * q(3) + gyro(2) * q(2));
    dq2 = 0.5f * (gyro(0) * q(3) + gyro(1) * q(0) - gyro(2) * q(1));
    dq3 = 0.5f * (-gyro(0) * q(2) + gyro(1) * q(1) + gyro(2) * q(0));
    return Quaternionf(dq0, dq1, dq2, dq3);
}



Quaternionf mahonyFromRef(IMUData_t imu, Quaternionf q, const cv::Matx31f& acc_ref, const cv::Matx31f& mag_ref)
{
    using namespace cv;
    const float Kp = 2.0f;
    Matx33f rm = q.toRotationMatrix();
    Matx31f a = rm * imu.accel;
    Matx31f m = rm * imu.mag;
    Matx31f e = cross(acc_ref, a) + cross(mag_ref, m);
    Matx31f g = imu.gyro + Kp * e;
    return (q * g).mul(0.5f);
}