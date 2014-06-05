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
		Eigen::Vector3f norm(0, 1, 2);
		Eigen::AngleAxisf rot(3.14f / 30.f, Eigen::Vector3f(0, 0, 1));
		hex::Serial uart("/dev/ttyAMA0");
		while(true) {
			p1.rotate(norm);
			p1.writeSerial(uart);
			norm = rot * norm;
			usleep(200000);
		}
	}
	catch (std::string eMsg) {
		std::cout << "Caught exception: " << eMsg << "\n";
	}
	return 0;
}

