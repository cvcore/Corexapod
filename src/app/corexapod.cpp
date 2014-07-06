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
	hex::Hexapod hexapod;
	hex::Parser p("src/actions.as", hexapod);
	hex::Daemon d(50000, p);
	d.spin();
}


