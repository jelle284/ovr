#include <iostream>
#include "opencv2/opencv.hpp"
#include "ovr.h"
#include <Windows.h>

using namespace cv;

int main(int argc, char* argv[]) {
	Quaternionf q = 0.75f*Quaternionf(1,2,3,4)*0.5f;
	for (int i = 0; i < 4; ++i)
		std::cout << q.val[i] << " ";
	std::cout << std::endl;
	return 0;
}

