/*
 * test.cpp
 *
 *  Created on: Jun 1, 2014
 *      Author: core
 */

#include "serial.h"
#include "engine.h"
#include <iostream>
#include <string>
#include <unistd.h>

int main() {
	hex::Plane p1;
	std::string eMsg;
	try {
		Eigen::Vector3f norm;
		float x, y, z;
		Eigen::AngleAxisf rot(3.14f / 30.f, Eigen::Vector3f(0, 0, 1));
		hex::Serial uart("/dev/ttyAMA0");
		while(true) {
//			std::cin >> x >> y >> z;
//			norm = Eigen::Vector3f(x, y, z);
//			p1.rotate(norm);
//			p1.writeSerial(uart);
//			std::cin.clear();
			for(float t = 0; t < 3.14 * 2; t += 3.14 / 20) {
				Eigen::Vector3f pos(150.8 + 5 * cos(t), 80.4 + 5 * sin(t), 0);
				p1.leg_[0]->setPosition(pos, p1);
				p1.writeSerial(uart);
				usleep(200000);
			}
		}
	}
	catch (std::string eMsg) {
		std::cout << "Caught exception: " << eMsg << "\n";
	}
	return 0;
	//test
}

