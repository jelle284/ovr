#include "viewer.h"

/* ---------------------------- VIEWER ---------------------------- */

Viewer::Viewer(int width, int height) :
	zoomdist(4.0), focallen(2400), width(width), height(height)
{
	using namespace cv;
	cm = Mat::zeros(3, 3, CV_64F);
	cm.at<double>(0, 0) = focallen;
	cm.at<double>(0, 2) = width / 2;
	cm.at<double>(1, 1) = focallen;
	cm.at<double>(1, 2) = height / 2;
	cm.at<double>(2, 2) = 1;
	rv = Mat::zeros(3, 1, CV_64F);
	rv_last = Mat::zeros(3, 1, CV_64F);
	tv = Mat::zeros(3, 1, CV_64F);
	dc = Mat::zeros(1, 5, CV_64F);
}

Viewer::~Viewer()
{
}

void Viewer::render_points(const std::vector<cv::Matx31d>& world, std::vector<cv::Point2d>& projected)
{
	cv::projectPoints(world, rv, tv, cm, dc, projected);
	for (auto& pt : projected) pt.y = height - pt.y;
}

void Viewer::mouse_handler(mouse_data_t mouse)
{
	using namespace cv;
	if (mouse.is_dragging) {
		Mat rv_delta = Mat::zeros(3, 1, CV_64F);
		const double scale = 0.005;
		rv_delta.at<double>(0, 0) = scale * ((double)mouse.yn - (double)mouse.ys);
		rv_delta.at<double>(1, 0) = scale * ((double)mouse.xn - (double)mouse.xs);
		Mat tv_zero = Mat::zeros(3, 1, CV_64F);
		cv::composeRT(
			rv_last, tv_zero,
			rv_delta, tv_zero,
			rv, tv_zero);
	}
	if (is_dragging_last && !mouse.is_dragging) {
		rv.copyTo(rv_last);
	}
	is_dragging_last = mouse.is_dragging;

	tv.at<double>(2) = zoomdist * (1 - double(mouse.scroll) / 1200);
}


/* ---------------------------- VIEWABLES ---------------------------- */

void Viewable::setColor(int r, int g, int b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

// COORDINATE SYSTEM
void CoordinateSystem::project(cv::Mat & im, Viewer& v)
{
	using namespace cv;
	std::vector<Matx31d> unit_sys, Tips;
	unit_sys.push_back(Point3d(0, 0, 0));
	unit_sys.push_back(Point3d(unit_len, 0, 0));
	unit_sys.push_back(Point3d(0, unit_len, 0));
	unit_sys.push_back(Point3d(0, 0, unit_len));
	Mat rmat;
	Rodrigues(this->rv, rmat);
	for (auto & axis : unit_sys) {
		Mat ax = rmat * Mat(axis) + this->tv;
		Tips.push_back(Point3d(ax));
	}
	std::vector<Point2d> tips;
	v.render_points(Tips, tips);
	std::vector<std::string> texts;
	texts.push_back(std::string("X"));
	texts.push_back(std::string("Y"));
	texts.push_back(std::string("Z"));

	std::vector<Scalar> colors;
	colors.push_back(Scalar(r, 0, 0));
	colors.push_back(Scalar(0, g, 0));
	colors.push_back(Scalar(0, 0, b));

	for (size_t i = 1; i < tips.size(); ++i) {
		line(im, tips.at(0), tips.at(i), colors.at(i - 1), thick);
		cv::putText(im, texts.at(i - 1), tips.at(i), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(255, 255, 255), thick);
	}
};

// ARROW
void Arrow::project(cv::Mat & im, Viewer& v)
{
	using namespace cv;
	std::vector<Matx31d> world;
	world.push_back(origin);
	world.push_back(origin + direction);
	std::vector<Point2d> projected;
	v.render_points(world, projected);
	arrowedLine(im, projected.at(0), projected.at(1), Scalar(r, g, b), thick);
}

// CIRCLE
void Circle::project(cv::Mat & im, Viewer& v)
{
	using namespace cv;
	std::vector<Matx31d> world;
	world.push_back(center);
	std::vector<Point2d> projected;
	v.render_points(world, projected);
	cv::circle(im, projected[0], 3, Scalar(r, g, b), 2);
}


