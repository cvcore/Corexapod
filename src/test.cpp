/*
 * test.cpp
 *
 *  Created on: Jun 1, 2014
 *      Author: core
 */

#include "serial.h"
#include "engine.h"

int main() {
	hex::Plane p1;
	hex::Serial uart("/dev/ttyAMA0");
	p1.rotate(Eigen::Vector3f(0, 0, 1));
	p1.writeSerial(uart);
	return 0;
}

