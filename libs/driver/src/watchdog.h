#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "common.h"

//-----------------------------------------------------------------------------
// Purpose: Watchdog
//-----------------------------------------------------------------------------
class COvrWatchdogDriver : public IVRWatchdogProvider
{
public:
	COvrWatchdogDriver()
	{
		m_pWatchdogThread = nullptr;
	}

	virtual EVRInitError Init( vr::IVRDriverContext *pDriverContext ) ;
	virtual void Cleanup() ;

private:
	std::thread *m_pWatchdogThread;
};

#endif // !WATCHDOG_H