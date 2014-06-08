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
#include <vector>
#include <sstream>
#include <iostream>

namespace hex {

typedef enum { left, right } SideType;

class Plane;
class Leg;

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
	int _minPW, _maxPW, _curPW, _speed, _number;
	float _angle;
	bool _changed;
};

class Leg {
public:
	Leg(SideType side, const Eigen::Vector3f& origin, const Eigen::Vector3f& pos, const std::vector<int>& servoNumberVec, const Plane& refPlane);
	~Leg();
	void setPosition(Eigen::Vector3f pos);
	void setOrigin(Eigen::Vector3f newOrigin);
	friend class Plane;

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
	void rotate(Eigen::Vector3f newNormal, float angle = 0.f);
	void translate(Eigen::Vector3f origin);
	void writeSerial(Serial& serial);
	Eigen::Vector3f projection(Eigen::Vector3f point) const;
	void calibrate();

	Leg* leg_[6];
	Eigen::Vector3f origin_, normal_;
	float roll_, pitch_, yaw_;
	Eigen::Vector3f initLegOrigin_[6], initLegPos_[6];
	Eigen::AngleAxisf rotater_;
private:
	Eigen::Vector3f _lastNormal;
};







}



#endif /* ENGINE_H_ */
