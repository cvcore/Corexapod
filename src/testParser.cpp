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
	hex::Parser p("src/actions.as", hexapod);
	std::cout << p;
	p.act(std::string("waveFrontLegs"));
	return 0;
}
