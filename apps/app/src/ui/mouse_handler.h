#ifndef MOUSE_HANDLER_H
#define MOUSE_HANDLER_H

#include "opencv2/opencv.hpp"

class mouse_data_t {
public:
	bool is_dragging;
	int scroll, xn, yn, xs, ys;
	mouse_data_t() : is_dragging(false), scroll(0), xn(0), yn(0), xs(0), ys(0) {}
	mouse_data_t operator()(cv::Rect roi) {
		mouse_data_t roi_mouse = *this;
		roi_mouse.yn -= roi.y;
		roi_mouse.ys -= roi.y;
		roi_mouse.xn -= roi.x;
		roi_mouse.xs -= roi.x;
		return roi_mouse;
	}
};

inline void on_mouse(int e, int x, int y, int d, void* ptr)
{
	using namespace cv;
	mouse_data_t* mouse = (mouse_data_t*)ptr;
	if (e == EVENT_LBUTTONDOWN) {
		mouse->is_dragging = true;
		mouse->xs = x;
		mouse->ys = y;
	}
	if (e == EVENT_MOUSEMOVE) {
		mouse->xn = x;
		mouse->yn = y;
	}
	if (e == EVENT_LBUTTONUP) {
		mouse->is_dragging = false;
	}
	if (e == EVENT_MOUSEWHEEL)
	{
		mouse->scroll += getMouseWheelDelta(d);
	}
}

#endif // !MOUSE_HANDLER_H