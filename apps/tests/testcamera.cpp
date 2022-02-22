#include "opencv2/opencv.hpp"
#include "ps3eye.h"
#include "camera.h"

void menu(Camera &cam) {
	cam.load();
	if (cam.isCalibrated())
		std::cout << "Calibration OK" << std::endl;
	else
		std::cout << "Not Calibrated" << std::endl;

	std::cout << "Camera: " << cam.getPortPath() << "\n"
		<< "1) View\n"
		<< "2) Adjust\n"
		<< "3) Calibrate Extrinsics\n"
		<< "4) Calibrate Intrinsics\n"
		<< "5) Save calibration\n"
		<< "0) Exit\n"
		<< std::endl;

	int iMode;
	bool bLoopStatus = true;
	while (bLoopStatus) {
		std::cout << "Input action: ";
		std::cin >> iMode;
		switch (iMode) {
		case 0:
			bLoopStatus = false;
			std::cout << std::endl;
			break;
		case 1:
			cam.view();
			break;
		case 2:
			cam.adjust();
			break;
		case 3:
			cam.calibrateChess();
			break;
		case 4:
			cam.calibrateIntrinsics();
			break;
		case 5:
			cam.save();
			break;
		default:
			std::cout << "Invalid choice! Try again." << std::endl;
			break;
		}
	}
}

void camerastream(Camera &cam, int t) {
	cam.load();
	cam.setTrackingStatus(true, false, false);
	std::cout << "---- Camera Test ----" << std::endl;
	std::cout << "ts, px, py " << std::endl;
	auto tbegin = std::chrono::system_clock::now();
	cam.start();
	while (1) {
		if ((tbegin + std::chrono::seconds(t)) < std::chrono::system_clock::now()) break;
		cam_update_t cam_data;
		while (cam.get_buffer(cam_data)) {
			auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(
				cam_data.timestamp.time_since_epoch());
			std::cout
				<< ts.count() << ","
				<< cam_data.pixel.x << ","
				<< cam_data.pixel.y << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	cam.stop();
}

int main(int argc, char *argv[]) {
	try {
		auto devices = ps3eye::PS3EYECam::getDevices();
		std::cout << devices.size() << " cameras found." << std::endl;
		for (auto& eye : devices) {
			Camera c(eye);
			menu(c);
			//camerastream(c, 20);
		}
	}
	catch (std::exception e) {
		std::cout << e.what();
	}
	return 0;
}