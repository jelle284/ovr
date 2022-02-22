#ifndef VIEWER_H
#define VIEWER_H

#include "opencv2/opencv.hpp"
#include "mouse_handler.h"

class Viewer {
public:
	Viewer(int width, int height);
	~Viewer();
	void render_points(const std::vector<cv::Matx31d>& world, std::vector<cv::Point2d>& projected);
	void mouse_handler(mouse_data_t mouse);
private:
	const double zoomdist, focallen;
	const int width, height;
	bool is_dragging_last;
	cv::Mat cm, dc, rv, tv, rv_last;
};

// Base class for a viewable object
class Viewable {
public:
	Viewable() : r(255), g(255), b(255) {}
	virtual void project(cv::Mat& im, Viewer& v) = 0;
	void setColor(int r, int g, int b);
protected:
	int r, g, b;
};

// viewable implementations
class CoordinateSystem : public Viewable {
	const double unit_len;
	const int thick;
	void init() {
		rv = cv::Matx31d::zeros();
		tv = cv::Matx31d::zeros();
	}
public:
	cv::Matx31d rv, tv;
	CoordinateSystem() :
		unit_len(1.0), thick(2) 
	{
		init();
	}
	CoordinateSystem(double unit_len, int thick) :
		unit_len(unit_len), thick(thick) 
	{
		init();
	}
	void project(cv::Mat& im, Viewer& v) override;
};

class Arrow : public Viewable {
	const int thick;
public:
	cv::Matx31d origin, direction;
	Arrow() : thick(1) {}
	Arrow(int thick) : thick(thick) {}
	void project(cv::Mat& im, Viewer& v) override;
};

class Circle : public Viewable {
	const int thick, radius;
public:
	cv::Matx31d center;
	Circle(int thick = 2, int radius = 3) : thick(thick), radius(radius) {}
	void project(cv::Mat& im, Viewer& v) override;
};

#endif // !VIEWER_H