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
		float x, y, z, w;
		Eigen::AngleAxisf rot(3.14f / 30.f, Eigen::Vector3f(0, 0, 1));
		hex::Serial uart("/dev/ttyAMA0");
//		while(true) {
//			for(float w = 0; w < 3.14 / 6; w += 3.14 / 6 / 20) {
//				p1.rotate(Eigen::Vector3f::UnitZ(), w);
//				p1.writeSerial(uart);
//				usleep(200000);
//			}
//		}
		p1.leg_[0]->setOrigin(Eigen::Vector3f(60, 59, 0));
		p1.writeSerial(uart);
	}
	catch (std::string eMsg) {
		std::cout << "Caught exception: " << eMsg << "\n";
	}
	return 0;
	//test
}

