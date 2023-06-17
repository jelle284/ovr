#ifndef HANDCONTROLLER_H
#define HANDCONTROLLER_H

#include "tracked_device.h"

class HandController : public TrackerBase {
public:
	HandController(EDevice tag);
	~HandController();

	// communation
	void parseUDP(char* buf, int buflen);
	bool is_assigned() { return chip_id > 0; }
	bool is_id(int check_id) { return chip_id == check_id; }
	void assign_chip(int assigned_id) { chip_id = assigned_id; }
	


	/* ovr interface */
	virtual void start() override;
	virtual void stop() override;
	virtual ADCData_t get_adc_data() override;
	virtual ButtonData_t get_button_data() override;
	virtual void teachButton(EButton button, int16_t value, int16_t thresh) override;

private:
	int chip_id;
	float val_buffer[13];
	float cvt_gyro, cvt_mag, cvt_acc;

	ButtonData_t m_buttons;
	ADCData_t m_adc;
	
	int16_t
		joy_x_min, joy_x_max, joy_x_mid,
		joy_y_min, joy_y_max, joy_y_mid,
		trig_min, trig_max,
		btn1_min, btn2_min, btn3_min, btn4_min,
		btn1_max, btn2_max, btn3_max, btn4_max;
	const int16_t deadzone;

	bool in_range(int16_t value, int16_t target, int16_t spread);
	double scale_range(int16_t value, int16_t min, int16_t max);
	double scale_range3(int16_t value, int16_t min, int16_t mid, int16_t max, int16_t deadzone);
	void button_update(int16_t* adc);

	/* tracked device interface */
	virtual void udef_write(cv::FileStorage& fs) const override;
	virtual void udef_read(const cv::FileNode& node) override;


};

#endif