#include "HandController.h"
#include <iostream>
#include <string>

void print_config(double* qzero, double* gravity, double* gyrobias, int16_t* btnconfig) {
	std::cout << "qzero: ";
	for (int i = 0; i < 3; ++i) std::cout << qzero[i] << ", ";
	std::cout << qzero[3] << std::endl;

	std::cout << "gravity: ";
	for (int i = 0; i < 2; ++i) std::cout << gravity[i] << ", ";
	std::cout << gravity[2] << std::endl;

	std::cout << "gyrobias: ";
	for (int i = 0; i < 2; ++i) std::cout << gyrobias[i] << ", ";
	std::cout << gyrobias[2] << std::endl;

	std::cout << "btn config";
	for (int i = 0; i < 15; ++i) std::cout << btnconfig[i] << ", ";
	std::cout << btnconfig[15] << std::endl;
}

void print_data(controller_update_t controller_data) {
	auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(controller_data.timestamp.time_since_epoch());
	std::cout
		<< ts.count() << ","
		<< controller_data.q[0] << ","
		<< controller_data.q[1] << ","
		<< controller_data.q[2] << ","
		<< controller_data.q[3] << ","
		<< controller_data.acc[0] << ","
		<< controller_data.acc[1] << ","
		<< controller_data.acc[2] << ","
		<< controller_data.ang_vel[0] << ","
		<< controller_data.ang_vel[1] << ","
		<< controller_data.ang_vel[2] << ","
		<< controller_data.aX << ","
		<< controller_data.aY << ","
		<< controller_data.trigger << ","
		<< controller_data.buttons << std::endl;
}

void controller_test(int t) {

	HandController hc;
	//hc.name = "Right Hand v2";
	hc.name = "Left Hand Controller";
	std::cout << "---- Controller Test ----" << std::endl;
	std::cout << "ts,qw,qx,qy,qz,ax,ay,az,gx,gy,gz,jx,jy,trig,btn" << std::endl;
	auto tbegin = std::chrono::system_clock::now();
	controller_update_t controller_data;
	hc.start();
	
	while (1) {
		if ((tbegin + std::chrono::seconds(t)) < std::chrono::system_clock::now()) break;
		if (hc.isConnected()) {
			while (hc.get_buffer(controller_data)) {
				print_data(controller_data);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
		else {
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
	}

	hc.stop();
}

int main(int argc, char *argv[]) {
	try {
		int t = 20;
		if (argc > 1) {
			t = std::stoi(argv[1]);
		}
		controller_test(t);
	}
	catch (std::exception e) {
		std::cout << e.what();
	}
	return 0;
}