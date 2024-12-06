
#include "udp_server.h"
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

			// get controller from map
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
				auto elapsed_since_last = system_clock::now() - timestamp_data[pController];
				pController->setConnected(elapsed_since_last < 1s);
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