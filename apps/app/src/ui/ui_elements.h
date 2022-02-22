#ifndef UI_ELEMENTS_H
#define UI_ELEMENTS_H

#include "mouse_handler.h"

#define IN_RECT(xp, yp, rect) (xp > rect.x && xp < (rect.x + rect.width) && yp > rect.y && yp < (rect.y + rect.height))
#define WIN_CLOSED(winname) (cv::getWindowProperty(winname, cv::WND_PROP_AUTOSIZE) == -1)

// Base class for UI elements
class ui_element {
public:
	ui_element() : has_changed(false) {}
	virtual void draw(cv::Mat& im) = 0;
	virtual void mouse_handler(mouse_data_t mouse) = 0;
	bool has_changed;
};
// Text Label
class static_text : public ui_element {
public:
	static_text(int x, int y, int width, int height);
	std::string text;
	virtual void draw(cv::Mat& im) override;
	virtual void mouse_handler(mouse_data_t mouse) override {}
private:
	const cv::Rect area;
	const uint margin;
};

// Double ended slider
class double_ended_slider : public ui_element {
public:
	double_ended_slider(int x, int y, int width, int height,
		std::string label,
		int min_value, int max_value);
	void mouse_handler(mouse_data_t mouse) override;
	void draw(cv::Mat& im) override;
	void set_values(int lower, int upper);
	int get_lower();
	int get_upper();
private:
	const std::string label;
	const cv::Rect area;
	cv::Rect lo_bar, filler, hi_bar;
	const int min_value, max_value;
	int start_lo, start_hi, prev_lo, prev_hi, lo_value, hi_value;
	bool drag_state_lo, drag_state_hi, is_dragging_last;
};

// Normal slider
class single_slider : public ui_element {
public:
	single_slider(int x, int y, int width, int height,
		std::string label,
		int min_value, int max_value);
	void mouse_handler(mouse_data_t mouse) override;
	void draw(cv::Mat& im) override;
	void set_value(int value);
	int get_value();
private:
	const std::string label;
	const cv::Rect area;
	cv::Rect bar;
	const int min_value, max_value;
	int start_value, prev_value, current_value;
	bool drag_state, is_dragging_last;
};

// Color picker with sphere indicator
class color_picker : public ui_element {
public:
	color_picker(int x, int y, int radius, cv::Rect roi);
	void mouse_handler(mouse_data_t mouse) override;
	void draw(cv::Mat& im) override;
	cv::Vec3b targetHSV;
private:
	const cv::Point center;
	const int radius;
	const cv::Rect roi;
	cv::Point pick;
	cv::Scalar color_rgb;
};

// Toggle button with green indicator
class toggle_button : public ui_element {
public:
	toggle_button(int x, int y, int width, int height, std::string label);
	void mouse_handler(mouse_data_t mouse) override;
	void draw(cv::Mat& im) override;
	bool get_state() { return toggle_state; }
	void set_state(bool state) { toggle_state = state; }
private:
	const std::string label;
	const cv::Rect area, lamp;
	bool push_state, toggle_state, is_dragging_last, prev_state;
};

class push_button : public ui_element {
public:
	push_button(int x, int y, int width, int height, std::string label);
	void mouse_handler(mouse_data_t mouse) override;
	void draw(cv::Mat& im) override;
	bool get_state() { return push_state; }
private:
	bool push_state;
	const std::string label;
	const cv::Rect area;
	bool prev_state;
};

// Option for grouping toggle buttons as radio button
class radio_buttons {
public:
	radio_buttons();
	void add(toggle_button* pbtn);
	void update();
	bool has_changed;
	toggle_button* get_current_btn() { return active_btn; }
private:
	std::vector<toggle_button*> buttons;
	void disable_all_but(toggle_button* btn);
	toggle_button* active_btn;
};

// Graph element
class graph_timeseries : public ui_element{
public:
	graph_timeseries(int x, int y, int width, int height, unsigned int n_max_points);
	void mouse_handler(mouse_data_t mouse) override {}
	void draw(cv::Mat& im) override;
	void add(double p, int color=0);
private:
	cv::Rect area;
	std::deque<double> graph_data[3];
	unsigned int n_max_points, margin;
	int ymin, ymax;
	cv::Point2d scaleToPoint(double d, unsigned int x);
};
#endif // !UI_ELEMENTS_H