#include <iostream>
#include "ui_elements.h"
#include "ovr.h"





// main
int main(int argc, char* argv[]) {
	// HELLO SANDBOX
	std::cout << "Hello Sandbox" << std::endl;

	// OVR Devices
	auto devices = getDevices();

	// Run GUI
	Window window("sandbox");
	
	ui::static_text 
		st(50, 50, 200, 50),
		rgyro(50, 150, 200, 400),
		lgyro(300, 150, 200, 400);

	st.text = "Hello";
	st.fontScale = 1.0;
	window.elements.push_back(&st);

	rgyro.text = "gyro placeholder";
	window.elements.push_back(&rgyro);

	lgyro.text = "gyro placeholder";
	window.elements.push_back(&lgyro);

	ui::toggle_button tb(700, 50, 200, 50, "toggle");
	window.elements.push_back(&tb);

	ui::push_button pb(700, 200, 200, 50, "Say Hi");
	window.elements.push_back(&pb);
	
	while (window.run(10)) {
		// toggle
		if (tb.has_changed) {
			std::cout << "toggle is " << (tb.get_state() ? "On" : "Off") << std::endl;
			for (auto d : devices) {
				if (d->getTag() == EDevice::Hmd) continue;
				tb.get_state() ? d->start() : d->stop();
			}
		}

		// push button
		if (pb.has_changed && pb.get_state()) {
			std::cout << "Hi from gui" << std::endl;
		}

		// write out data from devices
		for (auto d : devices) {
			if (d->has_data & EDataFlags::IMU) {
				auto imu_data = d->get_imu_data();
				auto pose_data = d->get_pose_data();
				auto adc_data = d->get_adc_data();
				auto btn_data = d->get_button_data();
				std::stringstream ss;
				ss << "gyro data\n" << imu_data.gyro << "\n";
				ss << "quaternion\n" << pose_data.q(0) << "\n" << pose_data.q(1) << "\n" << pose_data.q(2) << "\n" << pose_data.q(3) << "\n";
				ss << "adc\n" 
					<< adc_data.jx << "\n"
					<< adc_data.jy << "\n"
					<< adc_data.trig << "\n"
					<< adc_data.btn << "\n";	
				ss << "buttons\n"
					<< btn_data.joyXY << "\n"
					<< btn_data.trigger << "\n"
					<< btn_data.buttons << "\n";
				// update text box
				switch (d->getTag()) {
				case EDevice::RightHandController:
					rgyro.text = ss.str();
					break;
				case EDevice::LeftHandController:
					lgyro.text = ss.str();
					break;
				}
			}
		}
	}
	// End GUI
	std::cout << "Goodnight Sandbox" << std::endl;
	cv::FileStorage fs("sandbox.json", cv::FileStorage::WRITE);
	for (auto d : devices) {
		fs << d->get_name() << d;
	}
	return 0;
}
