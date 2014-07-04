/*
 * testParser.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: core
 */


#include "parser.hpp"
#include <fstream>
#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>

namespace qi = boost::spirit::qi;

int main() {
	hex::Hexapod hexapod;
	std::string method;
	hex::Parser p("src/actions.as", hexapod);
//	hexapod.sitDance();
	std::cout << p;
//	std::cin >> method;
	p.act("sitDance");
	return 0;
}
