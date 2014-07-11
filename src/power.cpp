/*
 * power.cpp
 *
 *  Created on: Jul 10, 2014
 *      Author: core
 */

extern "C" {
#include <wiringPi.h>
#include <wiringPiSPI.h>
}

#include "power.hpp"

const int servoPIN = 6;

PowerInterface::PowerInterface() {
	if(wiringPiSetup() == -1)
		std::cout << "[Power] wiringPISetup error.\n";
	pinMode(servoPIN, OUTPUT);
	digitalWrite(servoPIN, 0);
	servoPower_ = false;
	_adcValue = 0;
	if(wiringPiSPISetup(0, 500000) == -1)
		std::cout << "[Power] SPI init error.\n";
}

int PowerInterface::readBatteryPercentage() {
	unsigned char buf[3];
	const int lowest = 7.4 / 8.2 * 1024;
	buf[1] = 0b01101000; //MCP3002 channel 0, MSB first
	buf[0] = 0;
	wiringPiSPIDataRW(0, buf, 2);
	_adcValue = (int)buf[1] << 8 + buf[2];
	return (float)(_adcValue - lowest) / (1024 - lowest) * 100;
}

void PowerInterface::servoPower(bool on) {
	if(on) {
		digitalWrite(servoPIN, 1);
	} else {
		digitalWrite(servoPIN, 0);
	}
	servoPower_ = on;
}
