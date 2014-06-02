/*

 * engine.h
 *
 *  Created on: May 30, 2014
 *      Author: core
 */

#ifndef ENGINE_H_
#define ENGINE_H_

#include "Eigen/Dense"
#include <cmath>
#include <cassert>

namespace hex {

typedef enum { left, right } SideType;

class Plane;
class Leg;

class Servo {
public:
	Servo(int jointType, SideType side);
//	~Servo();
	void setPW(int pw);
	void setAngle(float angle);
	int getPW();
	friend class Leg;
private:
	int _minPW, _maxPW, _curPW, _speed;
	float _angle;
	bool _changed;
};

class Leg {
public:
	Leg(SideType side, Eigen::Vector3f origin, Eigen::Vector3f pos);
	~Leg();
	void setPosition(Eigen::Vector3f pos, Plane& refPlane);
	friend class Plane;

private:
	Servo* _servo[3];
	Eigen::Vector3f _origin, _pos, _initOrigin;
	SideType _side;
};

class Plane {
public:
	Plane();
	~Plane();
	void rotate(float roll, float pitch, float yaw);
	void rotate(Eigen::Vector3f newNormal);
//	void rotate(Eigen::Quaternion q);
	Eigen::Vector3f projection(Eigen::Vector3f point);

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
