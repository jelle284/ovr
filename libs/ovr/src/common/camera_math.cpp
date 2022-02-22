#include "camera_math.h"

cv::Matx31d castRay(const cv::Matx31d& rv, const cv::Matx33d& cm, const cv::Point2d& pix)
{
	using namespace cv;
	Matx31d Tip = cm * Matx31d(pix.x, pix.y, 1.0);
	Tip /= Tip(2);
	Matx33d rmat;
	Rodrigues(rv, rmat);
	Matx31d ray = rmat * Tip;
	return ray;
}

cv::Matx31d shortestPath(const cv::Matx31d& p1, const cv::Matx31d& d1, const cv::Matx31d& p2)
{
	using namespace cv;
	Matx31d d = d1 / norm(d1);
	Matx31d v = p2 - p1;
	double t = v.dot(d);
	Matx31d p3 = p1 + t * d;
	return p3 - p2;
}


cv::Vec3d intersect(const cv::Vec3d& p1, const cv::Vec3d& d1, const cv::Vec3d& p2, const cv::Vec3d& d2)
{
	using namespace cv;
	Vec3d n1 = d1.cross(d2.cross(d1));
	Vec3d n2 = d2.cross(d1.cross(d2));
	Vec3d p2_p1 = p2 - p1;
	Vec3d p1_p2 = p1 - p2;
	Vec3d c1 = p1 + p2_p1.dot(n2) / d1.dot(n2) * d1;
	Vec3d c2 = p2 + p1_p2.dot(n1) / d2.dot(n1) * d2;
	return (c1 + c2) / 2;
}