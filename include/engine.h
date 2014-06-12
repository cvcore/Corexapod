/*

 * engine.h
 *
 *  Created on: May 30, 2014
 *      Author: core
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include "serial.h"
#include "Eigen/Dense"
#include <cmath>
#include <cassert>
#include <limits>
#include <vector>
#include <list>
#include <queue>
#include <sstream>
#include <iostream>

namespace hex {

typedef enum { left, right } SideType;

class Plane;
class Leg;
struct Movement;

class Servo {
public:
	Servo(int jointType, SideType side, int number);
//	~Servo();
	void setPW(int pw);
	void setAngle(float angle);
	int getPW();
	friend class Leg;
	friend class Plane;
private:
	int _minPW, _maxPW, _curPW, _actTime, _number;
	float _angle;
	bool _changed;
};

class Leg {
public:
	Leg(SideType side, const Eigen::Vector3f& origin, const Eigen::Vector3f& pos, const std::vector<int>& servoNumberVec, const Plane& refPlane);
	~Leg();
	void setPosition(const Eigen::Vector3f& pos, int time = 0);
	void setOrigin(const Eigen::Vector3f& newOrigin);
	void resetMovement();
	void addMovement(const Eigen::Vector3f& position, int deltaT); //deltaT in ms
	Eigen::Vector3f requestPosition(int time) const; //time in ms

	friend class Plane;
	std::vector<Movement> moveGroup_;
private:
	Servo* _servo[3];
	Eigen::Vector3f _origin, _initOrigin, //related to plane origin
					_pos; //absolute
	SideType _side;
	const Plane& _refPlane;
};

class Plane {
public:
	Plane();
	~Plane();
	void rotate(float roll, float pitch, float yaw);
	void rotate(const Eigen::Vector3f& newNormal, float angle = 0.f);
	void translate(const Eigen::Vector3f& origin);
	void writeSerial(Serial& serial);
	Eigen::Vector3f projection(const Eigen::Vector3f& point) const;
	void calibrate();

	Leg* leg_[6];
	Eigen::Vector3f origin_, normal_;
	float roll_, pitch_, yaw_;
	Eigen::Vector3f initLegOrigin_[6], initLegPos_[6];
	Eigen::AngleAxisf rotater_;
};

class Hexapod {
public:
	Hexapod();
	void parseMovement();
	void moveLinear();
	Plane base_;
	Serial uart_;
};

struct Movement {
	Movement(const Eigen::Vector3f& position, int deltaT) : position_(position), deltaT_(deltaT) {}
	Eigen::Vector3f position_;
	int deltaT_;
};



}



#endif /* ENGINE_H_ */
