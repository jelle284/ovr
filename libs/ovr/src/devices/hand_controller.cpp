#include "hand_controller.h"

HandController::HandController(EDevice tag) :
	TrackerBase(tag),
	joy_x_min(0), joy_x_max(16383), joy_x_mid(8191),
	joy_y_min(0), joy_y_max(16383), joy_y_mid(8191),
	trig_min(0), trig_max(16383),
	btn1_min(0), btn2_min(2000), btn3_min(4000), btn4_min(6000),
	btn1_max(1000), btn2_max(3000), btn3_max(5000), btn4_max(7000),
	deadzone(100),
	chip_id(0), val_buffer{ 0.0f }, cvt_acc(1.0f), cvt_gyro(1.0f), cvt_mag(1.0f)
{
}

HandController::~HandController()
{
}

void HandController::start()
{
	m_running = true;
}

void HandController::stop()
{
	m_running = false;
}

void HandController::parseUDP(char* buf, int buflen)
{
	// determine message type
	uint8_t fc = buf[0];
	uint8_t msg_t = buf[1];
	uint8_t len = buf[2];
	uint8_t pnum = buf[3];
	
	switch (fc) {
	case 0: // device discovery
	{
		float* cvt = (float*)(buf + 8);
		cvt_acc = cvt[0];
		cvt_gyro = cvt[1];
		cvt_mag = cvt[2];
	}
	break;
	case 1: // data stream
	{
		int16_t* data = (int16_t*)(buf + 4);
		switch (msg_t) {
		case 2: // IMU
			//	WORLD	LOCAL
			//	X		X
			//	Y		-Z
			//	Z		Y
			val_buffer[0] = cvt_acc * data[0];
			val_buffer[1] = cvt_acc * -data[2];
			val_buffer[2] = cvt_acc * data[1];
			val_buffer[3] = cvt_gyro * data[3];
			val_buffer[4] = cvt_gyro * -data[5];
			val_buffer[5] = cvt_gyro * data[4];
			imu_update(val_buffer);
			break;
		case 3: // MAG
			//	IMU		MAG
			//	X		Y
			//	Y		X
			//	Z		-Z
			val_buffer[6] = cvt_mag * data[1];
			val_buffer[7] = cvt_mag * data[2];
			val_buffer[8] = cvt_mag * data[0];
			imu_update(val_buffer);
			break;
		case 4: // ADC
			button_update(data);
			break;
		}
		
		break;
	}
	} // switch (fc)
}

void HandController::button_update(int16_t* adc_data)
{
	std::lock_guard<std::mutex> lock(m_mtx);

	m_adc.jx = adc_data[0];
	m_adc.jy = adc_data[1];
	m_adc.trig = adc_data[2];
	m_adc.btn = adc_data[3];

	m_buttons.buttons = 0;
	if (in_range(m_adc.btn, btn1_min, btn1_max)) m_buttons.buttons = 1;
	else if (in_range(m_adc.btn, btn2_min, btn2_max)) m_buttons.buttons = 2;
	else if (in_range(m_adc.btn, btn3_min, btn3_max)) m_buttons.buttons = 3;
	else if (in_range(m_adc.btn, btn4_min, btn4_max)) m_buttons.buttons = 4;
	m_buttons.joyXY(0) = scale_range3(m_adc.jx, joy_x_min, joy_x_mid, joy_x_max, deadzone);
	m_buttons.joyXY(1) = scale_range3(m_adc.jy, joy_y_min, joy_y_mid, joy_y_max, deadzone);
	m_buttons.trigger = scale_range(m_adc.trig, trig_min, trig_max);
	// set flag
	has_data |= EDataFlags::IMU | EDataFlags::Button;
}

void HandController::teachButton(EButton button, int16_t value, int16_t thresh)
{
	switch (button) {

	case EButton::Btn1:
		btn1_min = value - thresh;
		btn1_max = value + thresh;
		break;
	case EButton::Btn2:
		btn2_min = value - thresh;
		btn2_max = value + thresh;
		break;
	case EButton::Btn3:
		btn3_min = value - thresh;
		btn3_max = value + thresh;
		break;
	case EButton::Btn4:
		btn4_min = value - thresh;
		btn4_max = value + thresh;
		break;
	case EButton::JxMid:
		joy_x_mid = value;
		break;
	case EButton::JxMin:
		joy_x_min = value;
		break;
	case EButton::JxMax:
		joy_x_max = value;
		break;
	case EButton::JyMid:
		joy_y_mid = value;
		break;
	case EButton::JyMin:
		joy_y_min = value;
		break;
	case EButton::JyMax:
		joy_y_max = value;
		break;
	case EButton::Tmin:
		trig_min = value;
		break;
	case EButton::Tmax:
		trig_max = value;
		break;
	}
}

bool HandController::in_range(int16_t value, int16_t min_value, int16_t max_value)
{
	return (value > min_value) && (value < max_value);
}

double HandController::scale_range(int16_t value, int16_t min_value, int16_t max_value)
{
	double d = 0.0;
	if (min_value > max_value) {
		int16_t range = min_value - max_value;
		d = (double)(min_value - value) / range;
	}
	else {
		int16_t range = max_value - min_value;
		d = (double)(value - min_value) / range;
	}
	return d;
}

double HandController::scale_range3(int16_t value, int16_t min_value, int16_t mid, int16_t max_value, int16_t deadzone)
{
	if (value > (mid + deadzone)) return scale_range(value, mid, max_value);
	if (value < (mid - deadzone)) return -scale_range(value, mid, min_value);
	return 0.0;
}

ButtonData_t HandController::get_button_data()
{
	std::lock_guard<std::mutex> lock(m_mtx);
	has_data &= ~EDataFlags::Button;
	return m_buttons;
}

void HandController::udef_write(cv::FileStorage& fs) const
{
	fs << "chip_id" << chip_id;

	fs << "joy_x_max" << joy_x_max;
	fs << "joy_x_min" << joy_x_min;
	fs << "joy_x_mid" << joy_x_mid;

	fs << "joy_y_max" << joy_y_max;
	fs << "joy_y_min" << joy_y_min;
	fs << "joy_y_mid" << joy_y_mid;

	fs << "trig_max" << trig_max;
	fs << "trig_min" << trig_min;

	fs << "btn1_min" << btn1_min;
	fs << "btn2_min" << btn2_min;
	fs << "btn3_min" << btn3_min;
	fs << "btn4_min" << btn4_min;

	fs << "btn1_max" << btn1_max;
	fs << "btn2_max" << btn2_max;
	fs << "btn3_max" << btn3_max;
	fs << "btn4_max" << btn4_max;

}

void HandController::udef_read(const cv::FileNode& node)
{
	node["chip_id"] >> chip_id;

	node["joy_x_max"] >> joy_x_max;
	node["joy_x_min"] >> joy_x_min;
	node["joy_x_mid"] >> joy_x_mid;

	node["joy_y_max"] >> joy_y_max;
	node["joy_y_min"] >> joy_y_min;
	node["joy_y_mid"] >> joy_y_mid;

	node["trig_max"] >> trig_max;
	node["trig_min"] >> trig_min;

	node["btn1_min"] >> btn1_min;
	node["btn2_min"] >> btn2_min;
	node["btn3_min"] >> btn3_min;
	node["btn4_min"] >> btn4_min;

	node["btn1_max"] >> btn1_max;
	node["btn2_max"] >> btn2_max;
	node["btn3_max"] >> btn3_max;
	node["btn4_max"] >> btn4_max;
}

ADCData_t HandController::get_adc_data()
{
	std::lock_guard<std::mutex> lock(m_mtx);
	return m_adc;
}