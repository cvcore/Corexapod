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

int main() {
	hex::Plane p1;
	try {
		hex::Serial uart("/dev/ttyAMA0");
		p1.rotate(Eigen::Vector3f(0, 0, 1));
		p1.writeSerial(uart);
	}
	catch (std::string eObj) {
		std::cerr << eObj << "\n";
	}
	return 0;
}

