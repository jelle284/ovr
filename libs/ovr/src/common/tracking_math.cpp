#include "tracking_math.h"

KalmanFilter3::KalmanFilter3() : wp(0.1), wm(0.1)
{
	using namespace cv;
	F = Matx66d::eye();
	B = Matx<double, 6, 3>::zeros();
	makeAB(1);
	H(0, 0) = 1.0;
	H(1, 1) = 1.0;
	H(2, 2) = 1.0;
	Pk = Matx66d::eye();
	Pk_ = Matx66d::eye();
	u = Matx31d::zeros();
	x = Matx61d::zeros();
	x_ = Matx61d::zeros();
}

void KalmanFilter3::makeAB(double dt)
{
	double ddt = 0.5 * dt * dt;

	F(0, 3) = dt;
	F(1, 4) = dt;
	F(2, 5) = dt;

	B(3, 0) = ddt;
	B(4, 1) = ddt;
	B(5, 2) = ddt;
}

void KalmanFilter3::predict()
{
    using namespace cv;
	x_ = F * x + B * u;
    Pk_ = F * Pk * F.t() + Matx66d::eye() * wp * wp;
}

void KalmanFilter3::correct(cv::Matx31d pos3d)
{
	using namespace cv;
	Matx31d y = pos3d - H * x_;
	Matx33d S = H * Pk_ * H.t() + Matx33d::eye() * wm * wm;
	Matx<double, 6, 3> K = Pk_ * H.t()*S.inv();

	x = x_ + K * y;
	Pk = (Matx66d::eye() - K * H)*Pk_;
}
