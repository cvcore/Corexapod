/*
 * engine.cpp
 *
 *  Created on: May 30, 2014
 *      Author: core
 */

#include "engine.h"

using namespace hex;
using namespace std;

const float PI = 3.14159265359f;
const float initHeight = 29.0f + 140.801278f;
const float leg1Len = 140.801278;
const float leg2Len = 86.0;

Servo::Servo(int jointType, SideType side, int number) {
	_speed = 100;
	_curPW = 1500;
	_changed = true;
	_number = number;
	//TODO: Add a self calibration method.
	if(side == left) {
		if(jointType % 3 == 2) {
			_minPW = 1500;
			_maxPW = 500;
			_angle = 0.f;
		} else {
			_minPW = 2500;
			_maxPW = 500;
			_angle = 90.f;
		}
	} else if(side == right) {
		if(jointType % 3 == 2) {
			_minPW = 1500;
			_maxPW = 500;
			_angle = 0.f;
		} else {
			_minPW = 500;
			_maxPW = 2500;
			_angle = 90.f;
		}
	} else {
		assert(false);
	}
}

void Servo::setAngle(float angle) {
	int lastPW = _curPW;
	_angle = angle;
	_curPW = (_maxPW - _minPW) * _angle / PI + _minPW;
	if(lastPW != _curPW)
		_changed = true;
}

Leg::Leg(SideType side, const Eigen::Vector3f& origin, const Eigen::Vector3f& pos, const std::vector<int>& servoNumberVec, const Plane& refPlane)
: _origin(origin), _initOrigin(origin), _pos(pos), _side(side), _refPlane(refPlane) {
	int sIdx;
	for(sIdx = 0; sIdx < 3; sIdx++) {
		_servo[sIdx] = new Servo(sIdx, side, servoNumberVec[sIdx]);
	}
}

Leg::~Leg() {
	for(int sIdx = 0; sIdx < 3; sIdx++) {
		delete _servo[sIdx];
	}
}

void Leg::setPosition(Eigen::Vector3f pos) {
	_pos = pos;
	Eigen::Vector3f pp = _refPlane.projection(pos),
					ol = _origin,
					lp = pp - (_origin + _refPlane.origin_),
					olp =  ol.cross(lp);
	Eigen::Vector3f initNormal(0, 0, 1);
	float alpha = asin(8.0f / lp.norm()),
		  beta = acos(ol.dot(lp) / (ol.norm() * lp.norm()));
	if(olp.dot(_refPlane.normal_) < 0) {
		beta = -beta;
	}
	if(_side == left)
		_servo[2]->setAngle(alpha - beta);
	else
		_servo[2]->setAngle(beta - alpha);

	Eigen::Vector3f ob = _origin.cross(_refPlane.normal_),
//					ob = _initOrigin.cross(initNormal),
					obOffset,
					bb,
					bd;
//	obOffset << 0, 0, refPlane.origin_(2) - 29.f;
	obOffset = _refPlane.origin_ + Eigen::Vector3f(0, 0, -29.0);
	float lbd, lbd2, theta, delta;
	const float lbc2 = leg2Len * leg2Len,
				lcd2 = leg1Len * leg1Len,
				lbc  = leg2Len,
				lcd  = leg1Len;

//	Eigen::AngleAxisf yaw(_servo[2]->_angle, Eigen::Vector3f(0, 0, 1));
	Eigen::AngleAxisf yaw(_servo[2]->_angle, _refPlane.origin_);
	ob = yaw * ob;
	if(_side == left) {
		ob = ob * 8.f / ob.norm();
	} else {
		ob = ob * -8.f / ob.norm();
	}
//	obOffset += _initOrigin;
	bb = ob + obOffset;
//	bb = refPlane.rotater_ * bb;
	bd = pos - bb;
	lbd = bd.norm();
	lbd2 = bd.squaredNorm();
	_servo[0]->setAngle(acos((lbc2 + lcd2 - lbd2) / (2 * lbc * lcd)));
	theta = acos((lbc2 + lbd2 - lcd2) / (2 * lbc * lbd));
	delta = acos(bd.dot(_refPlane.normal_) / bd.norm());
	_servo[1]->setAngle(delta - theta);
}

void Leg::setOrigin(Eigen::Vector3f newOrigin) {
	_origin = newOrigin;
	this->setPosition(_pos);
}

Plane::Plane()
: roll_(0), pitch_(0), yaw_(0), rotater_(0.f, Eigen::Vector3f(0, 0, 1)), origin_(0, 0, initHeight), normal_(0, 0, 1) {
	int legIdx;
	int servoNum;

	initLegOrigin_[0] << 75.0,	40.0,		0.0;
	initLegOrigin_[1] << 0.0,	67.8838,	0.0;
	initLegOrigin_[2] << -75.0,	40.0,		0.0;
	initLegOrigin_[3] << 75.0,	-40.0,		0.0;
	initLegOrigin_[4] << 0.0,	-67.8838,	0.0;
	initLegOrigin_[5] << -75.0,	-40.0,		0.0;

	initLegPos_[0] <<	150.8824,	80.4706,	0.0;
	initLegPos_[1] <<	0.0,		153.8388,	0.0;
	initLegPos_[2] <<	-158.8824,	80.4706,	0.0;
	initLegPos_[3] <<	150.8824,	-80.4706,	0.0;
	initLegPos_[4] <<	0.0,		-153.8388,	0.0;
	initLegPos_[5] <<	-158.8824,	-80.4706,	0.0;

	//left side
	for(servoNum = 21, legIdx = 0; legIdx < 3; legIdx++, servoNum += 3) {
		std::vector<int> servoVec;
		servoVec.push_back(servoNum);
		servoVec.push_back(servoNum + 1);
		servoVec.push_back(servoNum + 2);
		//initLegOrigin is related to origin_
		leg_[legIdx] = new Leg(left, initLegOrigin_[legIdx], initLegPos_[legIdx], servoVec, *this);
	}

	//right side
	for(servoNum = 12; legIdx < 6; legIdx++, servoNum -= 3) {
		std::vector<int> servoVec;
		servoVec.push_back(servoNum);
		servoVec.push_back(servoNum - 1);
		servoVec.push_back(servoNum - 2);
		leg_[legIdx] = new Leg(right, initLegOrigin_[legIdx], initLegPos_[legIdx], servoVec, *this);
	}
}

Plane::~Plane() {
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		delete leg_[legIdx];
	}
}

void Plane::rotate(float roll, float pitch, float yaw) {
	Eigen::Vector3f norm(0, 0, 1);
	Eigen::Matrix3f rM, pM, yM;
	rM <<	0				, 0				, 0				,
			0				, cos(roll)		, -sin(roll)	,
			0				, sin(roll)		, cos(roll)		;

	pM <<	cos(pitch)		, 0				, sin(pitch)	,
			0				, 1				, 0				,
			-sin(pitch)		, 0				, cos(pitch)	;

	yM <<	cos(yaw)		, -sin(yaw)		, 0				,
			sin(yaw)		, cos(yaw)		, 0				,
			0				, 0				, 1				;

	norm = rM * norm;
	norm = pM * norm;
	norm = yM * norm;

	this->rotate(norm);
}

void Plane::rotate(Eigen::Vector3f newNormal) {
	Eigen::Vector3f initNormal(0, 0, 1), rotate;
	float rotateAngle;
	newNormal.normalize();
	if(initNormal == newNormal) {
		rotateAngle = 0.f;
		rotate = initNormal;
	} else {
		rotate = initNormal.cross(newNormal);
		rotateAngle = acos(initNormal.dot(newNormal));
	}
	normal_ = newNormal;
	rotater_ = Eigen::AngleAxisf(rotateAngle, rotate);
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		leg_[legIdx]->setPosition(leg_[legIdx]->_pos);
	}
}

Eigen::Vector3f Plane::projection(Eigen::Vector3f point) const {
	Eigen::Vector3f op = point - origin_, result;
	float lProj = op.dot(normal_);
	result = point - lProj * normal_;
	return result;
}

void Plane::setOrigin(Eigen::Vector3f origin) {
	origin_ = origin;
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		leg_[legIdx]->setPosition(leg_[legIdx]->_pos);
	}
}

void Plane::writeSerial(Serial& serial) {
	int legIdx, servoIdx;
	std::stringstream ss;
	for(legIdx = 0; legIdx < 6; legIdx++) {
		for(servoIdx = 0; servoIdx < 3; servoIdx++) {
			if(leg_[legIdx]->_servo[servoIdx]->_changed) {
				ss << "#" << leg_[legIdx]->_servo[servoIdx]->_number <<
					  "P" << leg_[legIdx]->_servo[servoIdx]->_curPW <<
					  "T" << leg_[legIdx]->_servo[servoIdx]->_speed;
				leg_[legIdx]->_servo[servoIdx]->_changed = false;
			}
		}
	}
//	std::cout << ss.str() << std::endl;
	if(ss.str().size() != 0) {
		ss << "\r\n";
		serial.write(ss.str().c_str(), ss.str().size());
	}
}
