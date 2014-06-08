/*
 * serial.h
 *
 *  Created on: Jun 2, 2014
 *      Author: core
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef TEST
#include <iostream>
#endif

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include <string>
#include <iostream>
#include <cstddef>
#include <ctime>

namespace hex {

class Serial {
public:
	Serial(const char* path);
	~Serial();
	int write(const char *buffer, int size, float blockTime = 0.f);
	int read(char *buffer, int size);
	bool busy();
private:
	int _filedes;
	struct termios _options;
	std::clock_t _timestamp;
	bool _busy;
	float _blockTime;
};

}

#endif /* SERIAL_H_ */
