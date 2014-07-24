/*
 * uscdemo.cpp
 *
 *  Created on: Jul 23, 2014
 *      Author: core
 */

#include <uscwrapper.hpp>

int main() {
	initialize("/dev/ttyS0", "params.info");

	walk(0, 50, 1000, 5);
	std::cout << "-------\n";
	walk(55, 0, 1000, 5);
	std::cout << "-------\n";

	turn(3.14 / 15, 1000, 5);
	std::cout << "-------\n";
	turn(-3.14 / 15, 1000, 5);
	std::cout << "-------\n";

	return 0;
}
