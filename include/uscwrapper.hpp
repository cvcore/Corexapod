/*
 * uscwrapper.hpp
 *
 *  Created on: Jul 22, 2014
 *      Author: core
 */

#ifndef USCWRAPPER_HPP_
#define USCWRAPPER_HPP_

#include <engine.hpp>

void initialize(const char *paramFilePath, const char *uartPath);

void moveBody(float x, float y, float z, int time);

void walk(float x, float y, int count, int time);

void turn(float angle, int count, int time);

#endif /* USCWRAPPER_HPP_ */
