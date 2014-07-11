/*
 * hexapod.cpp
 *
 *  Created on: Jul 6, 2014
 *      Author: core
 */

#include "daemon.hpp"
#include "engine.hpp"
#include "parser.hpp"

int main() {
	hex::Hexapod hexapod("/dev/ttyAMA0", "src/calib.param");
	hex::Parser p("src/actions.as", hexapod);
	hex::Daemon d(50000, p);
	d.spin();
	return 0;
}
