#include "HeadMountDisplay.h"
#include <iostream>

void hmdstream(int t) {

	HeadMountDisplay hmd;
	hmd.name = "Head Mount Display";
	hmd.start();
	auto tbegin = std::chrono::system_clock::now();
	std::cout << "---- HMD Test ----" << std::endl;
	std::cout << "ts,ax,ay,az,gx,gy,gz,mx,my,mz" << std::endl;
	while (1) {
		if ((tbegin + std::chrono::seconds(t)) < std::chrono::system_clock::now()) break;
		if (hmd.isConnected()) {
			imu_update_t imu_data;
			while (hmd.get_imu_data(imu_data)) {
				auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
					imu_data.timestamp - tbegin);
					//imu_data.timestamp.time_since_epoch());
				std::cout
					<< ts.count() << ","
					<< imu_data.acc[0] << ","
					<< imu_data.acc[1] << ","
					<< imu_data.acc[2] << ","
					<< imu_data.gyro[0] << ","
					<< imu_data.gyro[1] << ","
					<< imu_data.gyro[2] << ","
					<< imu_data.mag[0] << ","
					<< imu_data.mag[1] << ","
					<< imu_data.mag[2] << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else {
			//std::cout << "hmd not connected" << std::endl;
			std::this_thread::sleep_for(std::chrono::seconds(2));
		}
	}

	hmd.stop();
}

int main(int argc, char *argv[]) {
	try {
		int t = 20;
		if (argc > 1) {
			t = std::stoi(argv[1]);
		}
		hmdstream(t);
	}
	catch (std::exception e) {
		std::cout << e.what();
	}
	return 0;
}