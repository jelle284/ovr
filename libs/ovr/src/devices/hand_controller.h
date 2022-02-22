#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

#include "tracked_device.h"

class HandController : public TrackerBase {
public:
	HandController(EDevice tag);
	~HandController();
	/* IDevice interface */
	void start() override;
	void stop() override;
	ButtonData_t get_button_data() override;
private:
	ButtonData_t m_buttons;
	std::mutex m_mtx;
	char* ip_addr;
	std::thread* pThread;
	int16_t
		joy_x_min, joy_x_max, joy_x_mid,
		joy_y_min, joy_y_max, joy_y_mid,
		trig_min, trig_max,
		btn1_min, btn2_min, btn3_min, btn4_min,
		btn1_max, btn2_max, btn3_max, btn4_max;
	const int16_t deadzone;

	void threadProc();
	bool in_range(int16_t value, int16_t target, int16_t spread);
	double scale_range(int16_t value, int16_t min, int16_t max);
	double scale_range3(int16_t value, int16_t min, int16_t mid, int16_t max, int16_t deadzone);
	void button_update(float* read_buf);
};

#endif