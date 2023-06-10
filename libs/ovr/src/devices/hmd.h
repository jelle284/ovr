#ifndef HEADMOUNTDISPLAY_H
#define HEADMOUNTDISPLAY_H

#include "tracked_device.h"

class HeadMountDisplay :
	public TrackerBase
{
private:
	std::thread* pThread;
	void threadFunc();
public:
	HeadMountDisplay();
	~HeadMountDisplay();

	/* ovr interface */
	virtual void start() override;
	virtual void stop() override;
};

#endif