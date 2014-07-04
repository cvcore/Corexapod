/*

 * engine.h
 *
 *  Created on: May 30, 2014
 *      Author: core
 */

#ifndef ENGINE_HPP_
#define ENGINE_HPP_

#include <Eigen/Dense>
#include <cmath>
#include <cassert>
#include <limits>
#include <vector>
#include <list>
#include <queue>
#include <sstream>
#include <iostream>
#include <fstream>
#include "serial.hpp"
//#include "parser.hpp"

namespace hex {

typedef enum { left, right } SideType;

class Plane;
class Leg;

struct Movement {
	Movement(const Eigen::Vector3f& position, int deltaT) : position_(position), deltaT_(deltaT) {}
	Eigen::Vector3f position_;
	int deltaT_;
};

struct Velocity {
	Velocity() : linear_(Eigen::Vector3f::Zero()), angular_(0.f) {}
	Eigen::Vector3f linear_;
	float angular_;
};

class Servo {
public:
	Servo(int jointType, SideType side, int number);
//	~Servo();
	void calibrate(Serial& serial);
	void setPW(int pw);
	void setAngle(float angle);
	int getPW();
	friend class Leg;
	friend class Plane;
private:
	int _minPW, _maxPW, _curPW, _actTime, _number, _jointType;
	float _angle;
	bool _changed;
};

class Leg {
public:
	Leg(SideType side, const Eigen::Vector3f& origin, const Eigen::Vector3f& pos, const std::vector<int>& servoNumberVec, const Plane& refPlane);
	~Leg();
	void setPosition(const Eigen::Vector3f& pos, int time = 500);
	void setOrigin(const Eigen::Vector3f& newOrigin, int time = 500);
	void step(const Eigen::Vector3f& unitDisp, int totalT, float height = 10.f);
	void turn(float unitAngularDisp, int totalT, float height = 10.f);
	void resetMovement();
	/* For Parser */
	void addMovement(const Eigen::Vector3f& position, int deltaT); //deltaT in ms
	Eigen::Vector3f requestPosition(int time) const; //time in ms

	friend class Plane;
	friend class Hexapod;
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
	Plane(const char *paramFilePath = NULL);
	~Plane();
	Eigen::Vector3f projection(const Eigen::Vector3f& point) const;
	void rotate(float roll, float pitch, float yaw, int time = 500);
	void rotate(const Eigen::Vector3f& newNormal, const Eigen::Vector3f& newFront, int time = 500);
	void rotateNorm(const Eigen::Vector3f& newNormal, int time = 500);
	void rotateFront(const Eigen::Vector3f& newFront, int time = 500);
	void translate(const Eigen::Vector3f& newOrigin, int time = 500);
	/* For Parser */
	void stepGroup(const Eigen::Vector3f& unitDisp, int stepT, const std::vector<int>& group, float height = 10.f);
	void turnGroup(float unitAngularDisp, int stepT, const std::vector<int>& group, float height = 10.f);
	void addRelMovementGroup(const Eigen::Vector3f& disp, int deltaT, const std::vector<int>& group);
	void resetMovementGroup(const std::vector<int>& group);
	void writeSerial(Serial& serial);
	void calibrate(Serial& serial);

	Leg* leg_[6];
	Eigen::Vector3f origin_, normal_, front_; //both normal_ and front_ should be normalized
//	float roll_, pitch_, yaw_;
	Eigen::Vector3f initLegOrigin_[6], initLegPos_[6];
	Eigen::AngleAxisf rotater_;
	Velocity vel_; //unit: mm/ms and rad/ms
};

class Hexapod {
public:
	Hexapod();
	void parseMovement();
	void moveLinear(Eigen::Vector3f unitDisp, int stepT, int count = 1);
	void moveAngular(float unitAngularDisp, int stepT, int count = 1);
	void calibrate();
	void rotateBodyAround(const Eigen::Vector3f& rotStartNorm, int totalT);

	Plane base_;
	Serial uart_;
private:
	std::string _actionFileContents;
	bool _actionFileContentsAvailiable;
};


}



#endif /* ENGINE_HPP_ */
