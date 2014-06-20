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
	try {
		hex::Hexapod hexapod;
//		hexapod.base_.leg_[0]->addMovement(Eigen::Vector3f(120, 50, 10), 500);
//		hexapod.base_.leg_[0]->addMovement(Eigen::Vector3f(150, 80, 0), 500);
//		hexapod.base_.leg_[1]->addMovement(Eigen::Vector3f(0, 123, 0), 500);
//		hexapod.base_.leg_[1]->addMovement(Eigen::Vector3f(0, 133, 10), 200);
//		hexapod.parseMovement();
		hexapod.base_.translate(Eigen::Vector3f(0, 0, 120));
//		hexapod.base_.rotate(Eigen::Vector3f(1, 0, 5), Eigen::Vector3f(5, 0, -1));
		hexapod.base_.writeSerial(hexapod.uart_);
		usleep(500000);
//		hexapod.calibrate();
		hexapod.moveLinear(Eigen::Vector3f(0, 30, 0), 1000, 5);
		hexapod.moveAngular(3.1415926 / 10, 1000, 5);
		hexapod.moveLinear(Eigen::Vector3f(65, 0, 0), 1000, 5);
		hexapod.waveFrontLegs(3000);
	} catch (std::string& eMsg) {
		std::cout << eMsg << '\n';
	}
}


//int main() {
//	hex::Plane p1;
//	std::string eMsg;
//	try {
//		Eigen::Vector3f norm(1, 0, 4);
//		float x, y, z, w;
//		std::string s;
//		char uartBuf[255];
//		Eigen::AngleAxisf rot(3.14f / 30.f, Eigen::Vector3f(0, 0, 1));
//		hex::Serial uart("/dev/ttyAMA0");
//		p1.leg_[0]->resetMovement();
//		p1.leg_[0]->addMovement(Eigen::Vector3f(65.f, 30.f, 0.f), 100);
//		p1.leg_[0]->addMovement(Eigen::Vector3f(55.f, 20.f, 10.f), 100);
//		std::cout << p1.leg_[0]->requestPosition(50) << '\n' << p1.leg_[0]->requestPosition(150) << '\n' << p1.leg_[0]->requestPosition(250) << '\n';
//
//	}
//	catch (std::string& eMsg) {
//		std::cout << "Caught exception: " << eMsg << "\n";
//	}
//	return 0;
//	//test
//}

