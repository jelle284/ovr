#include "hand_controller.h"
#include <WinSock2.h>


UDPServer::UDPServer() :
	running(false)
{
	pThread = new std::thread(&UDPServer::ThreadFunc, this);
}

UDPServer::~UDPServer()
{
	running = false;
	if (pThread) pThread->join();
	delete pThread;
}

void UDPServer::subscribe(HandController* pCl)
{
	m_subscribers.push_back(pCl);
}

void UDPServer::ThreadFunc()
{
	using namespace std::chrono;

	const char init_req[4] = { 0, 0, 0, 0 };
	const char stream_req[4] = { 1, 0, 0, 0 };
	const int port = 4210;

	WSADATA wsa;
	SOCKET sockfd;
	sockaddr_in si_broadcast;

	const int timeout_ms = 200;
	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = timeout_ms * 1000;

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		exit(EXIT_FAILURE);
	}

	sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&si_broadcast, 0, sizeof(si_broadcast));
	BOOL trueflag = 1;
	setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST,
		(char*)&trueflag, sizeof trueflag);
	si_broadcast.sin_family = AF_INET;
	si_broadcast.sin_port = htons(port);
	si_broadcast.sin_addr.s_addr = INADDR_BROADCAST;
	bind(sockfd, (struct sockaddr*)&si_broadcast, sizeof(si_broadcast));

	// mapping
	std::map<ULONG, HandController*> map_ctrl;
	std::map<HandController*, ULONG> map_addr;
	std::map<HandController*, system_clock::time_point> timestamp_data;
	std::map<HandController*, system_clock::time_point> timestamp_stream;

	// timing setup
	auto timestamp_discover = system_clock::now();
	auto timestamp_led = system_clock::now();
	const auto interval_discover = 500ms;
	const auto interval_led = 500ms;
	const auto interval_stream = 500ms;

	// thread loop
	running = true;
	while (running) {

		// sockets
		fd_set rfds, wfds;
		FD_ZERO(&rfds);
		FD_SET(sockfd, &rfds);
		FD_ZERO(&wfds);
		FD_SET(sockfd, &wfds);

		// Read data
		if (select(0, &rfds, NULL, NULL, &timeout) < 0) continue;
		if (FD_ISSET(sockfd, &rfds)) {
			int slen = sizeof(sockaddr);
			char read_buf[20];
			sockaddr_in  si_other;
			memset(&si_other, 0, sizeof(si_other));
			memset(read_buf, '\0', sizeof(read_buf));
			int bytes_read = recvfrom(sockfd, read_buf, sizeof(read_buf), 0, (sockaddr*)&si_other, &slen);
			ULONG other = si_other.sin_addr.s_addr;
			HandController* pController = map_ctrl[other];

			// parse message
			uint8_t fc = read_buf[0];
			uint8_t msg_t = read_buf[1];
			if (fc == 0 && msg_t == 0) {
				int chip_id = *(int*)(read_buf + 4);
				// check if chip is one of our controllers
				for (auto ch : m_subscribers) {
					if (ch->is_assigned() && ch->is_id(chip_id)) {
						pController = ch;
						break;
					}
				}
				// if not found then assign to first unassigned controller
				if (!pController) {
					for (auto ch : m_subscribers) {
						if (!ch->is_assigned()) {
							ch->assign_chip(chip_id);
							pController = ch;
							break;
						}
					}
				}
			}
			// run controller
			if (pController) {
				map_ctrl[other] = pController;
				map_addr[pController] = other;
				pController->parseUDP(read_buf, sizeof(read_buf));

				if (fc == 1) {
					timestamp_data[pController] = system_clock::now();
				}
			}
		}
		
		// Send data
		if (select(0, NULL, &wfds, NULL, &timeout) < 0) continue;
		if (FD_ISSET(sockfd, &wfds)) {
			// find controllers
			auto discover_elapsed = system_clock::now() - timestamp_discover;
			if (discover_elapsed > interval_discover) {
				sendto(sockfd, init_req, sizeof(init_req), 0, (sockaddr*)&si_broadcast, sizeof(si_broadcast));
				timestamp_discover = system_clock::now();
			}

			// toggle streaming
			for (auto ch : m_subscribers) {
				auto elapsed_since_stream = system_clock::now() - timestamp_stream[ch];
				if (elapsed_since_stream < 1s) continue;
				auto elapsed_since_data = system_clock::now() - timestamp_data[ch];
				bool should_start = (ch->isRunning() && elapsed_since_data > 1s);
				bool should_stop = (!ch->isRunning() && elapsed_since_data < 1s);
				
				if (should_start || should_stop) {
					auto si = si_broadcast;
					si.sin_addr.s_addr = map_addr[ch];
					sendto(sockfd, stream_req, sizeof(stream_req), 0, (sockaddr*)&si, sizeof(si));
					timestamp_stream[ch] = system_clock::now();
				}
			}

			// send some led data periodically to controller
			auto led_elapsed = system_clock::now() - timestamp_led;
			if (led_elapsed > interval_led) { 
				for (auto ch : m_subscribers) {
					auto si = si_broadcast;
					si.sin_addr.s_addr = map_addr[ch];
					char led_req[] = { 2,0,0,0, ch->led_R, ch->led_G, ch->led_B, ch->led_tmr };
					sendto(sockfd, led_req, sizeof(led_req), 0, (sockaddr*)&si, sizeof(si));
				}
				timestamp_led = system_clock::now();
			}
		}
	}
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
			val_buffer[0] = cvt_acc * data[0];
			val_buffer[1] = cvt_acc * data[1];
			val_buffer[2] = cvt_acc * data[2];
			val_buffer[3] = cvt_gyro * data[3];
			val_buffer[4] = cvt_gyro * data[4];
			val_buffer[5] = cvt_gyro * data[5];
			imu_update(val_buffer);
			break;
		case 3: // MAG
			val_buffer[6] = cvt_mag * data[0];
			val_buffer[7] = cvt_mag * data[1];
			val_buffer[8] = cvt_mag * data[2];
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
	else if (in_range(m_adc.btn, btn4_min, btn4_max)) m_buttons.buttons = 3;
	m_buttons.joyXY(0) = scale_range3(m_adc.jx, joy_x_min, joy_x_mid, joy_x_max, deadzone);
	m_buttons.joyXY(1) = scale_range3(m_adc.jy, joy_y_min, joy_y_mid, joy_y_max, deadzone);
	m_buttons.trigger = scale_range(m_adc.trig, trig_min, trig_max);
	// set flag
	has_data |= EDataFlags::IMU | EDataFlags::Button;
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

HandController::HandController(EDevice tag) :
	TrackerBase(tag),
	joy_x_min(0), joy_x_max(16383), joy_x_mid(8191),
	joy_y_min(0), joy_y_max(16383), joy_y_mid(8191),
	trig_min(0), trig_max(16383),
	btn1_min(0), btn2_min(2000), btn3_min(4000), btn4_min(6000),
	btn1_max(1000), btn2_max(3000), btn3_max(5000), btn4_max(7000),
	deadzone(100),
	led_R(255), led_G(255), led_B(255), led_tmr(1),
	chip_id(0), val_buffer{ 0.0f }, cvt_acc(1.0f), cvt_gyro(1.0f), cvt_mag(1.0f)
{
}

HandController::~HandController()
{
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