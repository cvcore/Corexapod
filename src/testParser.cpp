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
	hexapod.base_.translate(Eigen::Vector3f(0, 0, 120));
	hexapod.syncServoWithDelay(500);
	while(std::cin>>method) {
		if(method == "w")
			hexapod.moveLinear(Eigen::Vector3f(65, 0, 0), 1000, 3);
		else if(method == "s")
			hexapod.moveLinear(Eigen::Vector3f(-65, 0, 0), 1000, 3);
		else if(method == "a")
			hexapod.moveAngular(PI/10, 1000, 2);
		else if(method == "d")
			hexapod.moveAngular(-PI/10, 1000, 2);
		else
			p.act(method);
	}
	return 0;
}
