#ifndef UDPSERVER_H
#define UDPSERVER_H
#include "hand_controller.h"

class UDPServer {
	std::vector<HandController*> m_subscribers;
	bool running;
	std::thread* pThread;
	void ThreadFunc();
public:
	UDPServer();
	~UDPServer();
	void subscribe(HandController* pCl);
};

#endif