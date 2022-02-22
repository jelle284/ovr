#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include "opencv2/opencv.hpp"
#include <mutex>

class ObjectTracker {
public:
	cv::Vec3b lower_bound, upper_bound;
	bool use_morph, invert_hue, found, status;
	cv::Rect roi;
	cv::Mat mask;

	ObjectTracker();
	void update(const cv::Mat& im);
	cv::Point2d getPoint();
	// filestorage implement
	void write(cv::FileStorage& fs) const;
	void read(const cv::FileNode& node);
private:
	std::mutex m_mtx;
	cv::Matx44d A, Pk, Pk_;
	cv::Matx41d x, x_;
	cv::Matx<double, 2, 4> C;
	int roiSize;
	cv::Point roiCenter;
	std::chrono::system_clock::time_point m_tlast;

	bool detect(cv::Point2d& pixel, const cv::Mat& im);
};

static void write(cv::FileStorage& fs, const std::string&, const ObjectTracker& objTracker) {
	objTracker.write(fs);
}

static void read(const cv::FileNode& node, ObjectTracker& objTracker, const ObjectTracker&) {
	if (!node.empty())
		objTracker.read(node);
}

#endif