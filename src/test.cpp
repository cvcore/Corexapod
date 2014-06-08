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
		Eigen::Vector3f norm(1, 0, 4);
		float x, y, z;
		Eigen::AngleAxisf rot(3.14f / 30.f, Eigen::Vector3f(0, 0, 1));
		hex::Serial uart("/dev/ttyAMA0");
		while(true) {
//			p1.rotate(norm);
//			p1.writeSerial(uart);
//			std::cin.clear();
//			norm = rot * norm;
//			usleep(250000);

//			for(float t = 0; t < 3.14 * 2; t += 3.14 / 20) {
//				Eigen::Vector3f pos(150.8 + 50 * cos(t), 80.4 + 50 * sin(t), 0);
//				p1.leg_[0]->setPosition(pos);
//				p1.writeSerial(uart);
//				usleep(200000);
//			}

//			for(float t = 0; t < 3.14 * 2; t += 3.14 / 20) {
//				p1.translate(Eigen::Vector3f(30 * cos(t), 30 * sin(t), 100));
//				p1.writeSerial(uart);
//				usleep(250000);
//			}

			std::cin >> x >> y >> z;
			p1.translate(Eigen::Vector3f(x, y, z));
			p1.writeSerial(uart);
			usleep(200000);
		}
	}
	catch (std::string eMsg) {
		std::cout << "Caught exception: " << eMsg << "\n";
	}
	return 0;
	//test
}

