/*
 * serial.cpp
 *
 *  Created on: Jun 2, 2014
 *      Author: core
 */

#include "serial.h"

using namespace hex;

Serial::Serial(const char* path) {
	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively

	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.

	_filestream = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
	if(_filestream == -1) {
		throw std::string("Cannot open uart port!");
	}

	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//  *For USC32, use 9600
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	tcgetattr(_filestream, &_options);
	_options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	_options.c_iflag = IGNPAR;
	_options.c_oflag = 0;
	_options.c_lflag = 0;
	tcflush(_filestream, TCIFLUSH);
	tcsetattr(_filestream, TCSANOW, &_options);
}

Serial::~Serial() {
	if(close(_filestream) == -1)
		throw std::string("Cannot close");
}

//TODO: extend this member function
int Serial::read(char *buffer, int size) {
	return ::read(_filestream, (void *)buffer, (size_t)size);
}

int Serial::write(const char *buffer, int size) {
	int idx;
	for(idx = 0; idx < size; idx++)
		std::cout << static_cast<char>(buffer[idx]);
	return ::write(_filestream, (const void *)buffer, (size_t)size);
}


bool Serial::idle() {
	return true;
}
