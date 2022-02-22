#ifndef HEADMOUNTDISPLAY_H
#define HEADMOUNTDISPLAY_H

#include "tracked_device.h"

class HeadMountDisplay :
	public TrackerBase
{
private:
	void threadFunc();
	std::thread* pThread;
public:
	HeadMountDisplay();
	~HeadMountDisplay();
	
	void start() override;
	void stop() override;
};

#endif