#include "hand_controller.h"
#include <WinSock2.h>

void HandController::threadProc()
{
	const char init_req[4] = { 1, REG_NAME, 24, 0 };
	const char data_req[4] = { 1, 0, 12, 0 };
	const int port = 4210;

	WSADATA wsa;
	SOCKET s_broadcast, s_controller;
	sockaddr_in si_broadcast, si_controller;

	timeval timeout;
	timeout.tv_sec = 0;
	timeout.tv_usec = 100 * 1000;
	unsigned int drop_counter = 0;

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
		exit(EXIT_FAILURE);
	}

	s_broadcast = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&si_broadcast, 0, sizeof(si_broadcast));
	BOOL trueflag = 1;
	setsockopt(s_broadcast, SOL_SOCKET, SO_BROADCAST,
		(char*)&trueflag, sizeof trueflag);
	si_broadcast.sin_family = AF_INET;
	si_broadcast.sin_port = htons(port);
	si_broadcast.sin_addr.s_addr = INADDR_BROADCAST;

	s_controller = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	memset(&si_controller, 0, sizeof(si_controller));
	si_controller.sin_family = AF_INET;
	si_controller.sin_port = htons(port);
	si_controller.sin_addr.s_addr = INADDR_NONE;

	m_threadState = true;

	while (m_threadState) {
		fd_set fds;
		if (m_connectionStatus) {
			/* READ DATA */
			auto poll_deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(m_pollRate);
			sendto(s_controller, data_req, sizeof(data_req), 0, (sockaddr*)&si_controller, sizeof(si_controller));
			FD_ZERO(&fds);
			FD_SET(s_controller, &fds);
			if (select(0, &fds, NULL, NULL, &timeout) > 0) {
				int slen = sizeof(sockaddr);
				float read_buf[12];
				memset(read_buf, 0, sizeof(read_buf));
				int bytes_read = recvfrom(s_controller, (char*)read_buf, sizeof(read_buf), 0, (sockaddr*)&si_controller, &slen);
				if (bytes_read == sizeof(read_buf)) {
					drop_counter = 0;
					auto timestamp = std::chrono::system_clock::now();
					imu_update(read_buf);
					button_update(read_buf);
				} else { // incorrect number of bytes recieved
					++drop_counter;
					
				}
			} else { //timeout on select
				++drop_counter;
			}
			if (drop_counter > 5) m_connectionStatus = false;
			std::this_thread::sleep_until(poll_deadline);
		}
		else {
			/* SEARCH */
			FD_ZERO(&fds);
			FD_SET(s_broadcast, &fds);
			sendto(s_broadcast, init_req, sizeof(init_req), 0, (sockaddr*)&si_broadcast, sizeof(si_broadcast));

			while (select(0, &fds, NULL, NULL, &timeout) > 0) {
				int slen = sizeof(sockaddr);
				int read_buf[24];
				sockaddr_in  si_other;
				memset(&si_other, 0, sizeof(si_other));
				memset(read_buf, '\0', sizeof(read_buf));
				int bytes_read = recvfrom(s_broadcast, (char*)read_buf, sizeof(read_buf), 0, (sockaddr*)&si_other, &slen);
				char* pname = (char*)&read_buf[0];
				if ((bytes_read == sizeof(read_buf)) && (get_name().compare(pname) == 0)) {
					ip_addr = inet_ntoa(si_other.sin_addr);
					si_controller.sin_addr = si_other.sin_addr;

					int16_t* btnconfig = (int16_t*)&read_buf[REG_JMIN-init_req[1]];
					joy_x_min	 = btnconfig[0];
					joy_y_min	 = btnconfig[1];
					joy_x_max	 = btnconfig[2];
					joy_y_max    = btnconfig[3];
					joy_x_mid	 = btnconfig[4];
					joy_y_mid	 = btnconfig[5];
					trig_min	 = btnconfig[6];
					trig_max	 = btnconfig[7];

					m_connectionStatus = true;
				}

			}
		}
	}
}

void HandController::button_update(float* read_buf)
{
	std::lock_guard<std::mutex> lock(m_mtx);

	int16_t* adc_data = (int16_t*)&read_buf[REG_FLAGS];
	m_buttons.buttons = adc_data[0];
	m_buttons.joyXY(0) = scale_range3(adc_data[2], joy_x_min, joy_x_mid, joy_x_max, deadzone);
	m_buttons.joyXY(1) = scale_range3(adc_data[3], joy_y_min, joy_y_mid, joy_y_max, deadzone);
	m_buttons.trigger = scale_range(adc_data[4], trig_min, trig_max);
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
	int16_t range = max_value - min_value;
	d = (double)(value - min_value) / range;
	return d;
}

double HandController::scale_range3(int16_t value, int16_t min_value, int16_t mid, int16_t max_value, int16_t deadzone)
{
	if (value > (mid + deadzone)) return scale_range(value, mid, max_value);
	if (value < (mid - deadzone)) return scale_range(value, mid, min_value);
	return 0.0;
}

HandController::HandController(EDevice tag) :
	TrackerBase(tag),
	joy_x_min(0), joy_x_max(16383), joy_x_mid(8191),
	joy_y_min(0), joy_y_max(16383), joy_y_mid(8191),
	trig_min(0), trig_max(16383),
	btn1_min(0), btn2_min(2000), btn3_min(4000), btn4_min(6000),
	btn1_max(1000), btn2_max(3000), btn3_max(5000), btn4_max(7000),
	pThread(nullptr),
	deadzone(100)
{
}

HandController::~HandController()
{
}

void HandController::start()
{
	pThread = new std::thread(&HandController::threadProc, this);
}

void HandController::stop()
{
	if (m_threadState) {
		m_threadState = false;
		if (pThread) {
			pThread->join();
			delete pThread;
			pThread = nullptr;
		}
	}
}

ButtonData_t HandController::get_button_data()
{
	std::lock_guard<std::mutex> lock(m_mtx);
	has_data &= ~EDataFlags::Button;
	return m_buttons;
}
