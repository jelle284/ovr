#include "madgwick.h"

void gradient_descent_step(const float* q, const float* u, const float* y, float* c)
{
	/*
	performs gradient descent calculations
	q = array of 4
	u = array of 3
	y = array of 3
	*/
	c[0] = (2 * q[1] * u[1] - 2 * q[2] * u[0]) * ((-2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[1] + (2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[0] + u[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]) - y[2]) + (-2 * q[1] * u[2] + 2 * q[3] * u[0]) * ((2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[2] + (-2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[0] + u[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) - y[1]) + (2 * q[2] * u[2] - 2 * q[3] * u[1]) * ((-2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[2] + (2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[1] + u[0] * (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) - y[0]);
	c[1] = (-2 * q[2] * u[1] - 2 * q[3] * u[2]) * ((-2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[2] + (2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[1] + u[0] * (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) - y[0]) + (2 * q[0] * u[1] + 4 * q[1] * u[2] - 2 * q[3] * u[0]) * ((-2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[1] + (2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[0] + u[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]) - y[2]) + (-2 * q[0] * u[2] + 4 * q[1] * u[1] - 2 * q[2] * u[0]) * ((2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[2] + (-2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[0] + u[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) - y[1]);
	c[2] = (-2 * q[1] * u[0] - 2 * q[3] * u[2]) * ((2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[2] + (-2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[0] + u[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) - y[1]) + (-2 * q[0] * u[0] + 4 * q[2] * u[2] - 2 * q[3] * u[1]) * ((-2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[1] + (2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[0] + u[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]) - y[2]) + (2 * q[0] * u[2] - 2 * q[1] * u[1] + 4 * q[2] * u[0]) * ((-2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[2] + (2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[1] + u[0] * (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) - y[0]);
	c[3] = (-2 * q[1] * u[0] - 2 * q[2] * u[1]) * ((-2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[1] + (2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[0] + u[2] * (1 - 2 * q[1] * q[1] - 2 * q[2] * q[2]) - y[2]) + (2 * q[0] * u[0] - 2 * q[2] * u[2] + 4 * q[3] * u[1]) * ((2 * q[0] * q[1] + 2 * q[2] * q[3]) * u[2] + (-2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[0] + u[1] * (1 - 2 * q[1] * q[1] - 2 * q[3] * q[3]) - y[1]) + (-2 * q[0] * u[1] - 2 * q[1] * u[2] + 4 * q[3] * u[0]) * ((-2 * q[0] * q[2] + 2 * q[1] * q[3]) * u[2] + (2 * q[0] * q[3] + 2 * q[1] * q[2]) * u[1] + u[0] * (1 - 2 * q[2] * q[2] - 2 * q[3] * q[3]) - y[0]);
}

Quaternionf madgwickFilterUpdate(IMUData_t IMUData, Quaternionf q, const cv::Matx31f acc_ref, const cv::Matx31f mag_ref, const float beta)
{
	using namespace cv;
	Matx41f dfa, dfm, df;
	gradient_descent_step(q.val, IMUData.accel.val, acc_ref.val, dfa.val);
	gradient_descent_step(q.val, IMUData.mag.val, mag_ref.val, dfm.val);
	df = dfa + dfm;
	auto norm = cv::norm(df);
	Quaternionf dq = (q.mul(0.5f) * IMUData.gyro);
	if (norm > 0) dq = dq - beta*df/norm;
	return dq;
}