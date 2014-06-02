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

Servo::Servo(int jointType, SideType side) {
	_speed = 100;
	_curPW = 1500;
	_changed = true;
	if(side == left) {
		if(jointType % 3 == 2) {
			_minPW = 1500;
			_maxPW = 3500;
			_angle = 0.f;
		} else {
			_minPW = 2500;
			_maxPW = 500;
			_angle = 90.f;
		}
	} else if(side == right) {
		if(jointType % 3 ==1) {
			_minPW = 1500;
			_maxPW = 3500;
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

Leg::Leg(SideType side, Eigen::Vector3f origin, Eigen::Vector3f pos) : _origin(origin), _initOrigin(origin), _pos(pos), _side(side) {
	int sIdx;
	for(sIdx = 0; sIdx < 3; sIdx++) {
		_servo[sIdx] = new Servo(sIdx, side);
	}
}

Leg::~Leg() {
	for(int sIdx = 0; sIdx < 3; sIdx++) {
		delete _servo[sIdx];
	}
}



void Leg::setPosition(Eigen::Vector3f pos, Plane& refPlane) {
	Eigen::Vector3f pp = refPlane.projection(pos),
					ol = _origin - refPlane.origin_,
					lp = pp - _origin,
					olp =  ol.cross(lp);
	Eigen::Vector3f initNormal(0, 0, 1);
	float alpha = asin(8.0f / lp.norm()),
		  beta = acos(ol.dot(lp) / (ol.norm() * lp.norm()));
	if(olp.dot(refPlane.normal_) < 0) {
		beta = -beta;
	}
	if(_side == left)
		_servo[2]->setAngle(beta - alpha);
	else
		_servo[2]->setAngle(alpha - beta);

	Eigen::Vector3f ob = _initOrigin.cross(initNormal),
					obOffsetZ,
					bb,
					bd;
	obOffsetZ << 0, 0, -29.0 + initHeight;
	float lbd, lbd2, theta, delta;
	const float lbc2 = leg2Len * leg2Len,
				lcd2 = leg1Len * leg1Len,
				lbc  = leg2Len,
				lcd  = leg1Len;

	Eigen::AngleAxisf yaw(_servo[2]->_angle, Eigen::Vector3f(0, 0, 1));
	ob = yaw * ob;
	if(_side == left) {
		ob = ob * 8.f / ob.norm();
	} else {
		ob = ob * -8.f / ob.norm();
	}
	obOffsetZ += _initOrigin;
	bb = ob + obOffsetZ;
	bb = refPlane.rotater_ * bb;
	bd = _pos - bb;
	lbd = bd.norm();
	lbd2 = bd.squaredNorm();
	_servo[0]->setAngle(acos((lbc2 + lcd2 - lbd2) / (2 * lbc * lcd)));
	theta = acos((lbc2 + lbd2 - lcd2) / (2 * lbc * lbd));
	delta = acos(bd.dot(refPlane.normal_) / bd.norm());
	_servo[1]->setAngle(delta - theta);
}

Plane::Plane() : roll_(0), pitch_(0), yaw_(0), rotater_(0.f, Eigen::Vector3f(0, 0, 1)) {
	int legIdx;

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

	for(legIdx = 0; legIdx < 3; legIdx++) {
		leg_[legIdx] = new Leg(left, initLegOrigin_[legIdx], initLegPos_[legIdx]);
	}
	for(; legIdx < 6; legIdx++) {
		leg_[legIdx] = new Leg(right, initLegOrigin_[legIdx], initLegPos_[legIdx]);
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

	for(int legIdx = 0; legIdx < 6; legIdx++) {
		leg_[legIdx]->setPosition(leg_[legIdx]->_pos, *this);
	}
}

void Plane::rotate(Eigen::Vector3f newNormal) {
	Eigen::Vector3f initNormal(0, 0, 1);
	newNormal.normalize();
	Eigen::Vector3f rotate = initNormal.cross(newNormal);
	float rotateAngle = acos(initNormal.dot(newNormal));
	rotater_ = Eigen::AngleAxisf(rotateAngle, rotate);
}

Eigen::Vector3f Plane::projection(Eigen::Vector3f point) {
	Eigen::Vector3f op = point - origin_, result;
	float lProj = op.dot(normal_);
	result = point - lProj * normal_;
	return result;
}
