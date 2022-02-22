#include "object_tracker.h"

/************** Object tracker ****************/

ObjectTracker::ObjectTracker() :
	roiSize(200),
	lower_bound(0, 0, 0),
	upper_bound(179, 255, 255),
	use_morph(false), invert_hue(false),
	found(false), status(false)
{
	using namespace cv;
	const double dt = 0.016;
	A = cv::Matx44d::eye();
	A(0, 2) = dt;
	A(1, 3) = dt;
	C(0, 0) = 1.0;
	C(1, 1) = 1.0;
	Pk = Matx44d::eye();
	Pk_ = Matx44d::eye();
	x = Matx41d::zeros();
	x_ = Matx41d::zeros();
}

bool ObjectTracker::detect(cv::Point2d& pixel, const cv::Mat &im)
{

	using namespace cv;
	
	int imWidth = im.cols,
		imHeight = im.rows;

	const int RoiStep = 10,
		RoiMin = 60,
		RoiMax = max(imHeight, imWidth);

	// adjust ROI
	roi.x = max(roiCenter.x - roiSize, 0);
	roi.y = max(roiCenter.y - roiSize, 0);
	roi.width = min(2 * roiSize, imWidth - roi.x);
	roi.height = min(2 * roiSize, imHeight - roi.y);

	Mat HSV;
	cvtColor(im(roi), HSV, COLOR_BGR2HSV);
	if (invert_hue) {
		Mat m1, m2;
		inRange(HSV, 
			Scalar(0, lower_bound(1), lower_bound(2)),
			Scalar(lower_bound(0), upper_bound(1), upper_bound(2)),
			m1);
		inRange(HSV, 
			Scalar(upper_bound(0), lower_bound(1), lower_bound(2)),
			Scalar(179, upper_bound(1), upper_bound(2)),
			m2);
		mask = m1 | m2;
	}
	else {
		inRange(HSV, lower_bound, upper_bound, mask);
	}

	if (use_morph) {
		cv::erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
		cv::dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
	}
	
	Moments M = moments(mask, true);
	double x = (M.m10 / M.m00);
	double y = (M.m01 / M.m00);

	bool detected = (x > 0 && x < imWidth&& y > 0 && y < imHeight);
	if (detected)
	{ // if point is on image
		pixel.x = roi.x + x;
		pixel.y = roi.y + y;
		roiCenter = pixel;
		if (roiSize > RoiMin) roiSize -= RoiStep;
	}
	else
	{ // grow ROI
		if (roiSize < RoiMax) roiSize += RoiStep;
	}

	return detected;
}

void ObjectTracker::update(const cv::Mat& im)
{
	using namespace cv;
	const double vp = 0.1, vm = 0.1;

	Point2d pixel;
	if (detect(pixel, im)) {
		m_mtx.lock();
		// predict
		x_ = A * x;
		Pk_ = A * Pk * A.t() + Matx44d::eye() * vp;
		// correct
		Matx21d y = Matx21d(pixel.x, pixel.y) - C * x_;
		Matx22d S = C * Pk_ * C.t() + Matx22d::eye() * vm;
		Matx<double, 4, 2> K = Pk_ * C.t() * S.inv();
		x = x_ + K * y;
		Pk = (Matx44d::eye() - K * C) * Pk_;
		m_tlast = std::chrono::system_clock::now();
		m_mtx.unlock();
	}
	auto elapsed = std::chrono::duration<double>(std::chrono::system_clock::now() - m_tlast);
	found = (elapsed.count() < 1);
}

cv::Point2d ObjectTracker::getPoint()
{
	cv::Point2d p;
	m_mtx.lock();
	auto elapsed = std::chrono::duration<double>(std::chrono::system_clock::now() - m_tlast);
	p.x = x(0) + x(2) * elapsed.count();
	p.y = x(1) + x(3) * elapsed.count();
	m_mtx.unlock();
	return p;
}

void ObjectTracker::write(cv::FileStorage& fs) const
{
	fs << "{"
		<< "Lower bound" << lower_bound
		<< "Upper bound" << upper_bound
		<< "Morph" << use_morph
		<< "Invert" << invert_hue
		<< "}";
}

void ObjectTracker::read(const cv::FileNode& node)
{
	node["Lower bound"] >> lower_bound;
	node["Upper bound"] >> upper_bound;
	node["Morph"] >> use_morph;
	node["Invert"] >> invert_hue;
}

