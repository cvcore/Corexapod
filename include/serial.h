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

//FIXME naming pollution
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include <string>
#include <iostream>

namespace hex {

class Serial {
public:
	Serial(const char* path);
	~Serial();
	int write(const char *buffer, int size);
	int read(char *buffer, int size);
	bool idle();
private:
	int _filestream;
	struct termios _options;
};

}

#endif /* SERIAL_H_ */
