/*
 * power.hpp
 *
 *  Created on: Jul 10, 2014
 *      Author: core
 */

#ifndef POWER_HPP_
#define POWER_HPP_

#include <iostream>
#include <cstdlib>

/*
 * Note:
 * Firstly, you need to use the gpio utility to load the SPI drivers into the kernel:
 * gpio load spi
 * If you need a buffer size of greater than 4KB, then you can specify the size (in KB) on the command line:
 * gpio load spi 100
 * will allocate a 100KB buffer. (You should not need this though, the default is more than enough for most applications).
 *
 */

enum LogicPowerAction {poweroff, restart, shutdown};
enum ServoPowerAction {off = 0, on};

class PowerInterface {
public:
	PowerInterface();
	int readBatteryPercentage();
	void servoPower(ServoPowerAction action);
	void logicPower(LogicPowerAction action);
	bool servoPower_;
private:
	int _adcValue;
};


#endif /* POWER_HPP_ */
