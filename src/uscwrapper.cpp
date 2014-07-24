/*
 * uscwrapper.cpp
 *
 *  Created on: Jul 22, 2014
 *      Author: core
 */

#include "uscwrapper.hpp"
#include <boost/shared_ptr.hpp>

boost::shared_ptr<hex::Hexapod> phex;

void initialize(const char *uartPath, const char *paramFilePath) {
	phex = boost::shared_ptr<hex::Hexapod>( new hex::Hexapod(uartPath, paramFilePath) );
}

void moveBody(float x, float y, float z, int time) {
	Eigen::Vector3f newOrigin(x, y, z);
	newOrigin = newOrigin + phex->base_.origin_;
	phex->base_.translate(newOrigin, time);
	phex->syncServoWithDelay(time);
}

void walk(float x, float y, int time, int count) {
	Eigen::Vector3f disp(x, y, 0);
	phex->moveLinear(disp, time, count);
}

void turn(float angle, int time, int count) {
	phex->moveAngular(angle, time, count);
}
