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
	const int lowest = 7.4f * 1024 / 8.2;
	buf[0] = 0b11010000;//MCP3002 channel 0, MSB first
	buf[1] = 0;
	if(wiringPiSPIDataRW(0, buf, 2) == -1) {
		std::cout << "[Power] SPI rw error.\n";
		_adcValue = 1024;
	} else {
		_adcValue = ((buf[0] << 7) | (buf[1] >> 1)) & 0x3FF;
	}
	return (float)(_adcValue - lowest) / (1024 - lowest) * 100;
	//return _adcValue;
}

void PowerInterface::servoPower(ServoPowerAction action) {
	if(action == on) {
		digitalWrite(servoPIN, 1);
		servoPower_ = true;
	} else if(action == off) {
		digitalWrite(servoPIN, 0);
		servoPower_ = false;
	} else {
		std::cout << "[Power] Invalid operation\n";
	}
}

void PowerInterface::logicPower(LogicPowerAction action) {
	if(action == poweroff || action == halt) {
		std::system("shutdown -h now");
	} else if(action == restart) {
		std::system("reboot");
	} else {
		std::cout << "[Power] Invalid operation\n";
	}
}
