/*
 * uscengine.hpp
 *
 *  Created on: Jul 25, 2014
 *      Author: core
 */
#ifndef USCWRAPPER_HPP_
#define USCWRAPPER_HPP_

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <limits.h>

typedef struct {
	float x, y, z;
} Vector3f;

typedef struct {
	Vector3f linear;
	float angular;
} Velocity;

typedef enum {left, right} SideType;

const float PI = 3.14159265358979;

void initialize(const char *paramFilePath);

void walk(float x, float y, int count, int time);

void roll(float angle, int count, int time);

#endif /* USCWRAPPER_HPP_ */
