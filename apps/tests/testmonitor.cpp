#include <Windows.h>
#include <iostream>
#include <vector>



BOOL CALLBACK MonitorEnumProc(HMONITOR hMonitor,
    HDC      hdcMonitor,
    LPRECT   lprcMonitor,
    LPARAM   dwData)
{
    MONITORINFO info;
    info.cbSize = sizeof(info);
    if (GetMonitorInfo(hMonitor, &info))
    {
        std::vector<MONITORINFO>* monitors = (std::vector<MONITORINFO>*)dwData;
        monitors->push_back(info);

    }
    return TRUE;  // continue enumerating
}

int main()
{
    int count = 0;
    std::vector<MONITORINFO> monitors;
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, (LPARAM)&monitors);

    for (const auto &m : monitors) {
        std::cout << "Monitor: \n"
            << "x: " << std::abs(m.rcMonitor.left - m.rcMonitor.right)
            << " y: " << std::abs(m.rcMonitor.top - m.rcMonitor.bottom)
            << "\tLeft-top: (" << m.rcWork.left << ", " << m.rcWork.top << ")"
            << std::endl;
    }
    return 0;
}