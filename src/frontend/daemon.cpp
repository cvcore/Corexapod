/*
 * daemon.cpp
 *
 *  Created on: Jul 5, 2014
 *      Author: core
 */

#include "daemon.hpp"

using namespace hex;

Daemon::Daemon(int portNumber, Parser& parser) : _portno(portNumber), _parser(parser), _nonblock(false) {
	_sockfd = socket(AF_INET, SOCK_STREAM, 0); //If the third argument is zero (and it always should be except for unusual circumstances), the operating system will choose the most appropriate protocol. It will choose TCP for stream sockets and UDP for datagram sockets.
	if(_sockfd < 0)
		std::cout << "[Daemon] ERROR opening socket\n";

	memset(&_serv_addr, 0, sizeof(_serv_addr));
	_serv_addr.sin_family = AF_INET; // should always be AF_INET
	_serv_addr.sin_addr.s_addr = INADDR_ANY; //IP address of the machine on which the server is running.
	_serv_addr.sin_port = htons(_portno);

	if (bind(_sockfd, (struct sockaddr *) &_serv_addr, sizeof(_serv_addr)) < 0)
		std::cout << "[Daemon] ERROR on binding\n";
	listen(_sockfd, 5);
}

Daemon::~Daemon() {
	close(_sockfd);
}

void Daemon::spinOnce() {
	char buffer[255] = {0};
	struct sockaddr_in cli_addr;
	socklen_t clilen = sizeof(cli_addr);
	int newsockfd = accept(_sockfd, (struct sockaddr *) &cli_addr, &clilen);
	if (newsockfd < 0 || read(newsockfd, buffer, 255)< 0)
		if(!_nonblock)
			std::cout << "[Daemon] Socket communication error\n";
	std::cout << "[Debug] " << buffer << '\n';
	std::string result = _parser.parseSocket(buffer);
	write(newsockfd, result.c_str(), result.size());
	if(_parser.autoenable_)
		_parser.randomAct();
	if(_parser.autoenable_ && !_nonblock) {
		// set to nonblock socket mode
		int flags = fcntl(_sockfd, F_GETFL);
		if(flags == -1)
			std::cout << "[Daemon] F_GETFL error\n";
		flags |= O_NONBLOCK;
		if(fcntl(_sockfd, F_SETFL, flags))
			std::cout << "[Daemon] F_SETFL error\n";
		_nonblock = true;
	} else if(!_parser.autoenable_ && _nonblock){
		int flags = fcntl(_sockfd, F_GETFL);
		if(flags == -1)
			std::cout << "[Daemon] F_GETFL error\n";
		flags &= ~O_NONBLOCK;
		if(fcntl(_sockfd, F_SETFL, flags))
			std::cout << "[Daemon] F_SETFL error\n";
		_nonblock = false;
	}

	close(newsockfd);
}

void Daemon::spin() {
	while(!_parser.finish_)
		this->spinOnce();
}
