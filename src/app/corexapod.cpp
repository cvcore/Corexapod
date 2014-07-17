/*
 * hexapod.cpp
 *
 *  Created on: Jul 6, 2014
 *      Author: core
 */

#include <csignal>
#include <boost/bind.hpp>
#include "daemon.hpp"
#include "engine.hpp"
#include "parser.hpp"

hex::Hexapod hexapod("/dev/ttyAMA0", "src/calib.param");
hex::Parser p("src/actions.as", hexapod);
hex::Daemon d(50000, p);

void signalHandler(int sig) {
	p.finish_ = true;
}

int main() {
	std::signal(SIGTERM, signalHandler);
	d.spin();
	return 0;
}
