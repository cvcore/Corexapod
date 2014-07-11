/*
 * daemon.hpp
 *
 *  Created on: Jul 5, 2014
 *      Author: core
 */

#ifndef DAEMON_HPP_
#define DAEMON_HPP_

extern "C" {
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <memory.h>
#include <fcntl.h>
}

#include <iostream>
#include <ctime>
#include <fstream>
#include "parser.hpp"
#include "engine.hpp"

namespace hex {

class Daemon {
public:
	Daemon(int portNumber, Parser& parser);
	~Daemon();
	void spinOnce();
	void spin();
private:
	struct sockaddr_in _serv_addr, _cli_addr;
	int _portno, _sockfd;
	Parser& _parser;
	bool _nonblock;
};

}

#endif /* DAEMON_HPP_ */
