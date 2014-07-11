/*
 * power.hpp
 *
 *  Created on: Jul 10, 2014
 *      Author: core
 */

#ifndef POWER_HPP_
#define POWER_HPP_

#include <iostream>

/*
 * Note:
 * Firstly, you need to use the gpio utility to load the SPI drivers into the kernel:
 * gpio load spi
 * If you need a buffer size of greater than 4KB, then you can specify the size (in KB) on the command line:
 * gpio load spi 100
 * will allocate a 100KB buffer. (You should not need this though, the default is more than enough for most applications).
 *
 */

class PowerInterface {
	PowerInterface();
	int readBatteryPercentage();
	void servoPower(bool on);
	bool servoPower_;
private:
	int _adcValue;
};


#endif /* POWER_HPP_ */
