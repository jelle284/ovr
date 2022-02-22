#include "hmd.h"
#include <windows.h>

HANDLE hSerial = nullptr;
COMSTAT status;
DWORD errors;

void HeadMountDisplay::threadFunc()
{
	const char imu_req[4] = { 1, 0, 9, 0 };
	const char name_req[4] = { 1, REG_NAME, 8, 0 };
	m_threadState = true;
	while (m_threadState)
	{
		if (m_connectionStatus) { // read imu data from hmd
			auto poll_deadline = std::chrono::system_clock::now() + std::chrono::milliseconds(m_pollRate);
			if (WriteFile(hSerial, imu_req, sizeof(imu_req), 0, 0)) {
				float read_buf[9];
				DWORD bytesRead;
				if (ReadFile(hSerial, (char*)read_buf, sizeof(read_buf), &bytesRead, 0)) {
					if (bytesRead == sizeof(read_buf)) {
						imu_update(read_buf);
					}
				}
				else {
					m_connectionStatus = false;
				}
			}
			std::this_thread::sleep_until(poll_deadline);
		}
		else { // scan port 0 - 9 and check for a connected hmd
			for (int port = 0; port < 10; ++port) {
				std::string portname = "COM" + std::to_string(port);
				hSerial = CreateFile(
					portname.c_str(),
					GENERIC_READ | GENERIC_WRITE,
					0,
					NULL,
					OPEN_EXISTING,
					FILE_ATTRIBUTE_NORMAL,
					NULL);
				if (hSerial != INVALID_HANDLE_VALUE) {
					DCB dcbSerialParams = { 0 };
					if (GetCommState(hSerial, &dcbSerialParams)) {
						dcbSerialParams.BaudRate = CBR_115200;
						dcbSerialParams.ByteSize = 8;
						dcbSerialParams.StopBits = ONESTOPBIT;
						dcbSerialParams.Parity = NOPARITY;
						dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;
						dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;
						if (SetCommState(hSerial, &dcbSerialParams)) {
							PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
							std::this_thread::sleep_for(std::chrono::seconds(3));
							COMMTIMEOUTS comTimeOut = { 0 };
							comTimeOut.ReadTotalTimeoutConstant = 10;
							comTimeOut.WriteTotalTimeoutConstant = 10;
							SetCommTimeouts(hSerial, &comTimeOut);
							if (WriteFile(hSerial, name_req, sizeof(name_req), 0, 0)) {
								char read_buf[32] = { 0 };
								DWORD bytesRead;
								if (ReadFile(hSerial, (char*)read_buf, sizeof(read_buf), &bytesRead, 0)) {
									if ( (bytesRead == sizeof(read_buf)) && (get_name().compare(read_buf) == 0)) {
										m_connectionStatus = true;
									}
								}
							}
						}
					}
				}
				if (m_connectionStatus) break;
				CloseHandle(hSerial);
			}
		}
	}
	CloseHandle(hSerial);
}

HeadMountDisplay::HeadMountDisplay() :
	TrackerBase(EDevice::Hmd),
	pThread(nullptr)
{
	m_tag = EDevice::Hmd;
}

HeadMountDisplay::~HeadMountDisplay()
{

}

void HeadMountDisplay::start()
{
	pThread = new std::thread(&HeadMountDisplay::threadFunc, this);
}

void HeadMountDisplay::stop()
{
	m_threadState = false;
	if (pThread) {
		pThread->join();
		delete pThread;
		pThread = nullptr;
	}
}

