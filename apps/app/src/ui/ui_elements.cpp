#include "ui_elements.h"

using namespace ui;

/************** UI ELEMENTS ****************/
static const unsigned int barhandle_width = 8;
static const unsigned int text_offset = 11;
static const double text_small = 0.40;
static const double text_large = 0.60;
static const cv::Scalar shade_dark(100, 100, 100);
static const cv::Scalar shade_medium(120, 120, 120);
static const cv::Scalar shade_light(150, 150, 150);

static int ptov(int p, cv::Rect area, int max_value, int min_value) {
	return (max_value - min_value) * (float)(p - area.x) / area.width;
}
static int vtop(int v, cv::Rect area, int max_value, int min_value) {
	return area.x + area.width * (float)v / (max_value - min_value);
}

// STATIC TEXT
static_text::static_text(int x, int y, int width, int height) :
	area(x, y, width, height),
	margin(0.1 * width),
	fontScale(0.4)
{
}

void static_text::draw(cv::Mat& im)
{
	using namespace cv;
	const auto fontFace = FONT_HERSHEY_SIMPLEX;
	const int thickness = 1;
	const Scalar color(0, 0, 0);

	// draw area
	rectangle(im, area, shade_light, -1);

	// iterate over lines
	auto lines = std::vector<std::string>{};
	auto ss = std::stringstream{ text };
	for (std::string line; std::getline(ss, line, '\n');)
		lines.push_back(line);
	int line_count = 0;
	for (auto line : lines) {
		// center the text in the label
		int baseline = 0;
		Size textSize = getTextSize(line, fontFace,
			fontScale, thickness, &baseline);
		baseline += thickness;
		int line_space = 10;
		Point textOrg(area.x + (area.width - textSize.width) / 2,
			area.y + line_space + line_count*(textSize.height + line_space) + (textSize.height) / 2);
		// draw text
		putText(im, line, textOrg, fontFace, fontScale,
			color, thickness, 8);
		line_count++;
	}
}

// DOUBLE ENDED SLIDER
double_ended_slider::double_ended_slider(int x, int y, int width, int height,
	std::string label,
	int min_value, int max_value) :
	area(x, y, width, height),
	label(label),
	lo_bar(x, y, barhandle_width, height),
	filler(x + barhandle_width, y, width - 2 * barhandle_width, height),
	hi_bar(x + width - barhandle_width, y, barhandle_width, height),
	min_value(min_value), max_value(max_value), lo_value(min_value), hi_value(max_value),
	start_lo(min_value), start_hi(max_value), prev_lo(min_value), prev_hi(max_value),
	drag_state_lo(false), drag_state_hi(false), is_dragging_last(false)
{
}

void double_ended_slider::mouse_handler(mouse_data_t mouse)
{
	if (mouse.is_dragging) {
		if (!is_dragging_last) {
			drag_state_lo = IN_RECT(mouse.xs, mouse.ys, lo_bar);
			drag_state_hi = IN_RECT(mouse.xs, mouse.ys, hi_bar);
		}
		int x = mouse.xn - mouse.xs;
		int delta_v = ptov(area.x + x, area, max_value, min_value);
		if (drag_state_lo) {
			lo_value = start_lo + delta_v;
			if (lo_value < min_value) lo_value = min_value;
			if (lo_value > hi_value) lo_value = hi_value;
		}
		if (drag_state_hi) {
			hi_value = start_hi + delta_v;
			if (hi_value < lo_value) hi_value = lo_value;
			if (hi_value > max_value) hi_value = max_value;
		}
	}
	if (is_dragging_last && !mouse.is_dragging) {
		start_lo = lo_value;
		start_hi = hi_value;
	}
	is_dragging_last = mouse.is_dragging;
	has_changed = (prev_lo != lo_value || prev_hi != hi_value);
	prev_lo = lo_value;
	prev_hi = hi_value;
}

void double_ended_slider::draw(cv::Mat& im)
{
	lo_bar.x = vtop(lo_value, area, max_value, min_value) - barhandle_width;
	hi_bar.x = vtop(hi_value, area, max_value, min_value);
	filler.x = lo_bar.x + barhandle_width;
	filler.width = hi_bar.x - lo_bar.x - barhandle_width;
	using namespace cv;
	putText(im, label,
		Point(area.x + 0.25 * area.width, area.y - text_offset),
		FONT_HERSHEY_SIMPLEX, text_large, Scalar(0, 0, 0));
	rectangle(im, area, shade_light, -1);
	rectangle(im, lo_bar, shade_dark, -1);
	rectangle(im, filler, shade_medium, -1);
	rectangle(im, hi_bar, shade_dark, -1);
	putText(im, std::to_string(get_lower()),
		Point(lo_bar.x, lo_bar.y + area.height + text_offset),
		FONT_HERSHEY_SIMPLEX, text_small, Scalar(0, 0, 0));
	putText(im, std::to_string(get_upper()),
		Point(hi_bar.x, lo_bar.y + area.height + text_offset),
		FONT_HERSHEY_SIMPLEX, text_small, Scalar(0, 0, 0));
}

void double_ended_slider::set_values(int lower, int upper)
{
	if (lower < min_value) lower = min_value;
	if (upper > max_value) upper = max_value;
	if (lower > upper) lower = upper;
	if (upper < lower) upper = lower;
	lo_value = lower;
	hi_value = upper;
	start_lo = lower;
	start_hi = upper;

}

int double_ended_slider::get_lower()
{
	return lo_value;
}

int double_ended_slider::get_upper()
{
	return hi_value;
}

// SINGLE SLIDER
single_slider::single_slider(int x, int y, int width, int height,
	std::string label,
	int min_value, int max_value) :
	area(x, y, width, height), bar(x, y, barhandle_width, height),
	label(label),
	min_value(min_value), max_value(max_value), 
	start_value(min_value), prev_value(min_value), current_value(min_value),
	drag_state(false), is_dragging_last(false)
{
}

void single_slider::mouse_handler(mouse_data_t mouse)
{
	has_changed = false;
	if (mouse.is_dragging) {
		if (!is_dragging_last) {
			drag_state = IN_RECT(mouse.xs, mouse.ys, bar);
		}
		int x = mouse.xn - mouse.xs;
		int delta_v = ptov(area.x + x, area, max_value, min_value);
		if (drag_state) {
			current_value = start_value + delta_v;
			if (current_value < min_value) current_value = min_value;
			if (current_value > max_value) current_value = max_value;
		}
	}
	if (is_dragging_last && !mouse.is_dragging) {
		start_value = current_value;
	}
	is_dragging_last = mouse.is_dragging;
	has_changed = (current_value != prev_value);
	prev_value = current_value;
}

void single_slider::draw(cv::Mat& im)
{
	bar.x = vtop(current_value, area, max_value, min_value) - barhandle_width / 2;
	using namespace cv;
	putText(im, label,
		Point(area.x + 0.25 * area.width, area.y - text_offset),
		FONT_HERSHEY_SIMPLEX, text_large, Scalar(0, 0, 0));
	rectangle(im, area, shade_light, -1);
	rectangle(im, bar, shade_dark, -1);
	putText(im, std::to_string(get_value()),
		Point(bar.x, bar.y + area.height + text_offset),
		FONT_HERSHEY_SIMPLEX, text_small, Scalar(0, 0, 0));
}

void single_slider::set_value(int value)
{
	if (value > max_value) value = max_value;
	if (value < min_value) value = min_value;
	current_value = value;
	start_value = value;
}

int single_slider::get_value()
{
	return current_value;
}

// COLOR PICKER
color_picker::color_picker(int x, int y, int radius, cv::Rect roi) :
	center(x, y), radius(radius), roi(roi), color_rgb(0, 0, 0), targetHSV(0, 0, 0)
{
}

void color_picker::mouse_handler(mouse_data_t mouse)
{
	has_changed = false;
	if (mouse.is_dragging && IN_RECT(mouse.xs, mouse.ys, roi)) {
		pick.x = mouse.xs;
		pick.y = mouse.ys;
		has_changed = true;
	}
}

void color_picker::draw(cv::Mat& im)
{
	using namespace cv;
	if (has_changed) {
		Mat HSV;
		Mat roi = im(Rect(pick.x, pick.y, 1, 1));
		color_rgb = Scalar(roi.data[0], roi.data[1], roi.data[2]);
		cvtColor(roi, HSV, COLOR_BGR2HSV);
		targetHSV = HSV.at<Vec3b>(0, 0);
	}
	std::stringstream ss;
	ss << targetHSV;
	cv::circle(im, center, radius, color_rgb, -1);
	cv::circle(im, center, radius, Scalar(0, 0, 0), 1);
	putText(im, ss.str(),
		Point(center.x - 0.75 * radius, center.y + radius + text_offset),
		FONT_HERSHEY_SIMPLEX, text_small, Scalar(0, 0, 0));
}

// TOGGLE BUTTON
toggle_button::toggle_button(int x, int y, int width, int height, std::string label) :
	area(x, y, width, height),
	lamp(x + 0.02 * width, y + 0.1 * height, 0.08 * width, 0.8 * height),
	label(label),
	toggle_state(false), is_dragging_last(false), prev_state(false), push_state(false)
{
}

void toggle_button::mouse_handler(mouse_data_t mouse)
{
	push_state = (mouse.is_dragging && IN_RECT(mouse.xs, mouse.ys, area));
	if (push_state) {
		if (!is_dragging_last) {
			toggle_state = !toggle_state;
		}
	}
	is_dragging_last = mouse.is_dragging;
	has_changed = (prev_state != toggle_state);
	prev_state = toggle_state;
}

void toggle_button::draw(cv::Mat& im)
{
	using namespace cv;
	int t = 2;
	rectangle(im, area, shade_medium, t);
	Scalar fill_color = (push_state ? shade_medium : shade_light);
	rectangle(im, Rect(area.x + t, area.y + t, area.width - t, area.height - t), fill_color, -1);
	Scalar lamp_color = (toggle_state ? Scalar(0, 255, 0) : shade_medium);
	rectangle(im, lamp, lamp_color, -1);
	putText(im, label,
		Point(area.x + 0.2 * area.width, area.y + 0.67 * area.height),
		FONT_HERSHEY_SIMPLEX, text_large, Scalar(0, 0, 0));
}

// PUSH BUTTON
push_button::push_button(int x, int y, int width, int height, std::string label) :
	push_state(false), prev_state(false),
	area(x, y, width, height),
	label(label)
{

}

void push_button::mouse_handler(mouse_data_t mouse)
{
	push_state = (mouse.is_dragging && IN_RECT(mouse.xs, mouse.ys, area));
	has_changed = (prev_state != push_state);
	prev_state = push_state;
}

void push_button::draw(cv::Mat& im)
{
	using namespace cv;
	int t = 2;
	rectangle(im, area, shade_medium, t);
	Scalar fill_color = (push_state ? shade_medium : shade_light);
	rectangle(im, Rect(area.x, area.y + t, area.width - t, area.height - t), fill_color, -1);
	putText(im, label,
		Point(area.x + 0.2 * area.width, area.y + 0.67 * area.height),
		FONT_HERSHEY_SIMPLEX, text_large, Scalar(0, 0, 0));
}

//RADIO BUTTONS
radio_buttons::radio_buttons() :
	active_btn(nullptr), has_changed(false)
{
}

void radio_buttons::add(toggle_button* pbtn)
{
	if (buttons.empty()) {
		pbtn->set_state(true);
		active_btn = pbtn;
	}
	else pbtn->set_state(false);
	buttons.push_back(pbtn);
}

void radio_buttons::update() {
	for (auto& b : buttons) {
		if (b->has_changed) {
			if (b == active_btn) b->set_state(true);
			else if (b->get_state()) {
				disable_all_but(b);
				return;
			}
		}
	}
	has_changed = false;
}

void radio_buttons::disable_all_but(toggle_button* pbtn)
{
	for (auto& b : buttons) {
		if (b != pbtn) b->set_state(false);
	}
	active_btn = pbtn;
	has_changed = true;
}

// GRAPH TIMESERIES
graph_timeseries::graph_timeseries(int x, int y, int width, int height, unsigned int n_max_points) :
	n_max_points(n_max_points),
	ymin(0), ymax(1), margin(5),
	area(x, y, width, height)
{
}

void graph_timeseries::draw(cv::Mat& im)
{
	using namespace cv;
	// define colors
	const Scalar colors[] = { Scalar(0,0,255), Scalar(0,255,0), Scalar(255,0,0) };
	// clear area
	rectangle(im, area, Scalar(120, 120, 120), 5);
	rectangle(im, area, Scalar(40, 40, 40), -1);
	// loop colors
	for (int color = 0; color < 3; ++color) {
		auto dataseries = graph_data[color];
		// check we have at least 2 point for drawing lines
		if (dataseries.size() > 1) {
			// loop data points
			for (int i = 1; i < dataseries.size(); ++i) {
				double d = dataseries[i];
				// autoscale y axis
				if (d < ymin) ymin = d;
				if (d > ymax) ymax = d;
				// scale point to graph axis
				Point2d p = scaleToPoint(d, i);
				double prev_d = dataseries[i - 1];
				Point2d prev_point = scaleToPoint(prev_d, i - 1);
				// draw line from prev point to current;
				
				line(im, prev_point, p, colors[color]);
			}
		}
		// draw legend
		auto leg_start = cv::Point2d(area.x + color * 20, area.y + 10);
		line(im, leg_start, leg_start + Point2d(10, 0), colors[color]);
	}

}

void graph_timeseries::add(double d, int color)
{
	if (color >= 0 && color < 3) {
		graph_data[color].push_back(d);
		while (graph_data[color].size() > n_max_points) graph_data[color].pop_front();
	}
}

cv::Point2d graph_timeseries::scaleToPoint(double d, unsigned int x)
{
	cv::Rect r(area.x + margin, area.y + margin, area.width - 2 * margin, area.height - 2 * margin);
	double yscale = 0.5 * r.height / (ymax - ymin);
	double xscale = (double)r.width / n_max_points;
	double yval = r.y + 0.5 * r.height - d * yscale;
	double xval = r.x + x * xscale;
	return cv::Point2d(xval, yval);
}

// Main window
Window::Window(std::string name, cv::Scalar color) :
	winname(name),
	color(color)
{
	using namespace cv;
	namedWindow(winname);
	setMouseCallback(winname, on_mouse, &mouse);
	canvas = Mat::zeros(800, 1280, CV_8UC3);
}

Window::~Window()
{
}

bool Window::run()
{
	return run(16);
}

bool Window::run(int delay_ms) {
	// check window status
	if WIN_CLOSED(winname) {
		cv::destroyWindow(winname);
		return false;
	}

	// clear canvas
	rectangle(canvas, cv::Rect(0, 0, canvas.cols, canvas.rows), color, -1);

	// loop elements
	bool event = false;
	for (auto e : elements) {
		e->mouse_handler(mouse);
		e->draw(canvas);
		if (e->has_changed) event = true;
	}
	if (!event) {
		// show images
		cv::imshow(winname, canvas);
		int key = cv::waitKey(delay_ms);
	}
	return true;
}