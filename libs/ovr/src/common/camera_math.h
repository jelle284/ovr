#ifndef CAMERA_MATH_H
#define CAMERA_MATH_H

#include "opencv2/opencv.hpp"

/*
casts a ray from a camera, given rotation vector, camera matrix and pixel
returns 3d point of ray tip relative to camera position
*/
cv::Matx31d castRay(
	const cv::Matx31d& rv,
	const cv::Matx33d& cm,
	const cv::Point2d& pix);

/*
calculates shortest path from a point to a 3d line
	p1	:	point on line
	d1	:	direction of line
	p2	:	original position
	returns correction vector for p2
*/
cv::Matx31d shortestPath(
	const cv::Matx31d& p1,
	const cv::Matx31d& d1,
	const cv::Matx31d& p2);

/* 
Intersects two 3d lines given by point and direction
returns a point which has the smallest equal distance to both lines
*/
cv::Vec3d intersect(
	const cv::Vec3d& p1,
	const cv::Vec3d& d1,
	const cv::Vec3d& p2,
	const cv::Vec3d& d2);

#endif