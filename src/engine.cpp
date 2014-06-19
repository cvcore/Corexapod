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
	_actTime = 200; //ms
	_curPW = 1500;
	_changed = true;
	_number = number;
	_jointType = jointType;
	//TODO: Add a self calibration method.
	if(side == left) {
		if(jointType % 3 == 2) {
			_minPW = 1500;
			_maxPW = 500;
			_angle = 0.f;
		} else {
			_minPW = 2500;
			_maxPW = 500;
			_angle = PI / 2;
		}
	} else if(side == right) {
		if(jointType % 3 == 2) {
			_minPW = 1500;
			_maxPW = 500;
			_angle = 0.f;
		} else {
			_minPW = 500;
			_maxPW = 2500;
			_angle = PI / 2;
		}
	} else {
		assert(false);
	}
}

void Servo::calibrate(Serial& serial) {
	float ang1, ang2, pw1, pw2;
	std::string pwbuf;
	std::stringstream ss;
	std::cout << "#" << _number << "," << _jointType << ":\n";
	std::cout << "Angle 1(deg):";
	std::cin >> ang1;
	std::cout << "PW1(" << _minPW << "):";
	std::cin >> pwbuf;
	while(pwbuf != std::string("f")) {
		pw1 = atof(pwbuf.c_str());
		ss << "#" << _number << "P" << pw1 << "T" << 100 << "\r\n";
		serial.write(ss.str().c_str(), ss.str().size());
		std::cin >> pwbuf;
	}
	std::cout << "Angle 2(deg):";
	std::cin >> ang2;
	std::cout << "PW2(" << _maxPW << "):";
	std::cin >> pwbuf;
	while(pwbuf != std::string("f")) {
		pw2 = atof(pwbuf.c_str());
		ss << "#" << _number << "P" << pw2 << "T" << 100 << "\r\n";
		serial.write(ss.str().c_str(), ss.str().size());
		std::cin >> pwbuf;
	}

	ang1 = ang1 * PI / 180.f;
	ang2 = ang2 * PI / 180.f;
	_minPW = pw1 - (float)(pw2 - pw1) * ang1 / (ang2 - ang1);
	_maxPW = _minPW + (pw2 - pw1) * PI / (ang2 - ang1);
	this->setAngle(_angle);

	std::cout << "Servo " << _number << "finished calibration\n";
}

void Servo::setPW(int pw) {
	if(pw != _curPW) {
		_changed = true;
		_curPW = pw;
	}
}

void Servo::setAngle(float angle) {
	_angle = angle;
	this->setPW((_maxPW - _minPW) * _angle / PI + _minPW);
}

Leg::Leg(SideType side, const Eigen::Vector3f& origin, const Eigen::Vector3f& pos, const std::vector<int>& servoNumberVec, const Plane& refPlane)
: _origin(origin), _initOrigin(origin), _pos(pos), _side(side), _refPlane(refPlane) {
	int sIdx;
	for(sIdx = 0; sIdx < 3; sIdx++) {
		_servo[sIdx] = new Servo(sIdx, side, servoNumberVec[sIdx]);
	}
	this->resetMovement();
}

Leg::~Leg() {
	for(int sIdx = 0; sIdx < 3; sIdx++) {
		delete _servo[sIdx];
	}
}

void Leg::setPosition(const Eigen::Vector3f& pos, int time) {
	_pos = pos;
	Eigen::Vector3f pp = _refPlane.projection(pos),
					ol = _origin,
					lp = pp - (_origin + _refPlane.origin_),
					olp =  ol.cross(lp);
	Eigen::Vector3f initNormal(Eigen::Vector3f::UnitZ());
	float alpha = asin(8.0f / lp.norm()),
		  beta = acos(ol.dot(lp) / (ol.norm() * lp.norm()));
	if(olp.dot(_refPlane.normal_) < 0) {
		beta = -beta;
	}
	if(_side == left)
		_servo[2]->setAngle(beta + alpha);
	else
		_servo[2]->setAngle(beta - alpha);

	Eigen::Vector3f ob = _origin.cross(_refPlane.normal_),
//					ob = _initOrigin.cross(initNormal),
					obOffset,
					bb,
					bd;
//	obOffset << 0, 0, _refPlane.origin_(2) - 29.f;
//	obOffset = _refPlane.origin_ + _origin + Eigen::Vector3f(0, 0, -29.0);
	obOffset = _refPlane.origin_ + _origin + _refPlane.normal_ * (-29.0) / _refPlane.normal_.norm();
	float lbd, lbd2, theta, delta;
	const float lbc2 = leg2Len * leg2Len,
				lcd2 = leg1Len * leg1Len,
				lbc  = leg2Len,
				lcd  = leg1Len;

//	Eigen::AngleAxisf yaw(_servo[2]->_angle, Eigen::Vector3f(0, 0, 1));
	Eigen::AngleAxisf yaw(_servo[2]->_angle, _refPlane.normal_);
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

	_servo[0]->_actTime = _servo[1]->_actTime = _servo[2]->_actTime = time;
	if(time < 50)
		std::cout << "time interval: " << time << " too short, may cause problems\n";
}

void Leg::setOrigin(const Eigen::Vector3f& newOrigin) {
	_origin = newOrigin;
	this->setPosition(_pos);
}

void Leg::step(const Eigen::Vector3f& unitDisp, int totalT, float height) {
	this->resetMovement();
	this->addMovement(_pos + _refPlane.normal_ * height, totalT / 3);
	this->addMovement(_pos + unitDisp + _refPlane.normal_ * height, totalT / 3);
	this->addMovement(_pos + unitDisp, totalT / 3);
}

void Leg::turn(float unitAngularDisp, int totalT, float height) {
	this->resetMovement();
	Eigen::AngleAxisf rotater(unitAngularDisp, Eigen::Vector3f(0, 0, 1));
	Eigen::Vector3f newRelPos = rotater * (_pos - _refPlane.origin_),
					newPos = newRelPos + _refPlane.origin_;
	this->addMovement(_pos + _refPlane.normal_ * height, totalT / 3);
	this->addMovement(newPos + _refPlane.normal_ * height, totalT / 3);
	this->addMovement(newPos, totalT / 3);
}

void Leg::resetMovement() {
	moveGroup_.clear();
	moveGroup_.push_back(Movement(_pos, 0));
}

void Leg::addMovement(const Eigen::Vector3f& position, int deltaT) {
	moveGroup_.push_back(Movement(position, deltaT));
}

Eigen::Vector3f Leg::requestPosition(int time) const {
	std::vector<Movement>::const_iterator it = moveGroup_.begin();
	Eigen::Vector3f lastPosition(it->position_);
	int sumT = 0;
	for(it++; it != moveGroup_.end() && sumT + it->deltaT_ < time; sumT += it->deltaT_, lastPosition = it->position_, it++);
	if(it == moveGroup_.end())
		return lastPosition;
	else
		return (lastPosition + (it->position_ - lastPosition) * (float)(time - sumT) / it->deltaT_);
}

Plane::Plane(const char *paramFilePath)
: origin_(0, 0, initHeight), normal_(Eigen::Vector3f::UnitZ()), front_(Eigen::Vector3f::UnitX()) {
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

	if(paramFilePath != NULL) {
		std::ifstream paramFile(paramFilePath);
		int legIdx, sIdx;
		Servo* pServo;
		while(!paramFile.eof()) {
			paramFile >> legIdx >> sIdx;
			if(legIdx < 0 || legIdx >= 6 || sIdx < 0 || sIdx >= 3) {
				std::cout << "Invalid parameter, skipped\n";
				continue;
			}
			pServo = leg_[legIdx]->_servo[sIdx];
			paramFile >> pServo->_minPW >> pServo->_maxPW;
		}
	}
}

Plane::~Plane() {
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		delete leg_[legIdx];
	}
}

void Plane::rotate(float roll, float pitch, float yaw) {
	Eigen::Vector3f norm(Eigen::Vector3f::UnitZ());
	Eigen::Vector3f front(Eigen::Vector3f::UnitX());
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

	norm = (yM * (pM * (rM * norm)));
	front = (yM * (pM * (rM * front)));

	this->rotate(norm, front);
}

void Plane::rotate(const Eigen::Vector3f& newNormal, const Eigen::Vector3f& newFront) {
	Eigen::Vector3f initNormal(Eigen::Vector3f::UnitZ()), rotate;
	float rotateAngle;
	normal_ = newNormal; front_ = newFront;
	normal_.normalize(); front_.normalize();
	if(initNormal == normal_) {
		rotateAngle = 0.f;
		rotate = initNormal;
	} else {
		rotate = initNormal.cross(normal_);
		rotate.normalize();
		rotateAngle = acos(initNormal.dot(normal_));
	}

	Eigen::AngleAxisf rot0(rotateAngle, rotate);
	Eigen::Vector3f front0 = rot0 * Eigen::Vector3f::UnitX();
	float frontRotAng = acos(front0.dot(front_));
	if(front0.cross(front_).dot(normal_) < 0)
		frontRotAng = PI * 2 - frontRotAng;

	Eigen::AngleAxisf rotater0(frontRotAng, Eigen::Vector3f(0, 0, 1)), rotater;
	rotater = Eigen::AngleAxisf(rotateAngle, rotate);
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		leg_[legIdx]->setOrigin((Eigen::Vector3f)(rotater * (rotater0 * leg_[legIdx]->_initOrigin)));
	}
}

Eigen::Vector3f Plane::projection(const Eigen::Vector3f& point) const {
	Eigen::Vector3f op = point - origin_, result;
	float lProj = op.dot(normal_);
	result = point - lProj * normal_;
	return result;
}

void Plane::translate(const Eigen::Vector3f& newOrigin) {
	origin_ = newOrigin;
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		leg_[legIdx]->setPosition(leg_[legIdx]->_pos);
	}
}

void Plane::writeSerial(Serial& serial) {
	int legIdx, servoIdx;
	float maxTime = 0.f;
	std::stringstream ss;
	for(legIdx = 0; legIdx < 6; legIdx++) {
		for(servoIdx = 0; servoIdx < 3; servoIdx++) {
			if(leg_[legIdx]->_servo[servoIdx]->_changed) {
				leg_[legIdx]->_servo[servoIdx]->_changed = false;
				ss << "#" << leg_[legIdx]->_servo[servoIdx]->_number <<
					  "P" << leg_[legIdx]->_servo[servoIdx]->_curPW <<
					  "T" << leg_[legIdx]->_servo[servoIdx]->_actTime;
				if(leg_[legIdx]->_servo[servoIdx]->_actTime > maxTime * 1000)
					maxTime = (float)leg_[legIdx]->_servo[servoIdx]->_actTime / 1000.f;
			}
		}
	}
//	std::cout << ss.str() << std::endl;
	if(ss.str().size() != 0) {
		ss << "\r\n";
		serial.write(ss.str().c_str(), ss.str().size(), maxTime);
	}

}

void Plane::calibrate(Serial& serial) {
	std::ofstream conf("calib.conf");
	int legIdx, sIdx;
	for(legIdx = 0; legIdx < 6; legIdx++) {
		for(sIdx = 0; sIdx < 3; sIdx++) {
			Servo *pServo = leg_[legIdx]->_servo[sIdx];
			pServo->calibrate(serial);
			conf << legIdx << ' ' << sIdx << ' ' << pServo->_number << ' ' << pServo->_minPW << ' ' << pServo->_maxPW << ' ' << std::endl;
		}
	}
	conf.close();
}

void Plane::stepGroup(const Eigen::Vector3f& unitDisp, int stepT, const std::vector<int>& group, float height) {
	std::vector<int>::const_iterator it = group.begin();
	for(; it != group.end(); it++) {
		leg_[*it]->step(unitDisp, stepT, height);
	}
}

void Plane::turnGroup(float unitAngularDisp, int stepT, const std::vector<int>& group, float height) {
	std::vector<int>::const_iterator it = group.begin();
	for(; it != group.end(); it++) {
		leg_[*it]->turn(unitAngularDisp, stepT, height);
	}
}

void Plane::resetMovementGroup(const std::vector<int>& group) {
	std::vector<int>::const_iterator it = group.begin();
	for(; it != group.end(); it++)
		leg_[*it]->resetMovement();
}

Hexapod::Hexapod() : uart_("/dev/ttyAMA0") {}

void Hexapod::parseMovement() {
	int nextT[6] = {0}, next[6] = {1, 1, 1, 1, 1, 1};
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		if(base_.leg_[legIdx]->moveGroup_.size() == 1)
			nextT[legIdx] = std::numeric_limits<int>::max();
		else {
			nextT[legIdx] = base_.leg_[legIdx]->moveGroup_[1].deltaT_;
		}
	}
	int lastT = 0, currT = 0;
	for(int minPos = min_element(nextT, nextT + 6) - nextT;
			nextT[minPos] != std::numeric_limits<int>::max();
			minPos = min_element(nextT, nextT + 6) - nextT) {
			Leg *minLeg = base_.leg_[minPos];
		if(nextT[minPos] <= currT) {
			if((++next[minPos]) == minLeg->moveGroup_.size())
				nextT[minPos] = std::numeric_limits<int>::max();
			else
				nextT[minPos] += minLeg->moveGroup_[next[minPos]].deltaT_;
			continue;
		} else {
			currT = nextT[minPos];
			if((++next[minPos]) == minLeg->moveGroup_.size())
				nextT[minPos] = std::numeric_limits<int>::max();
			else
				nextT[minPos] += minLeg->moveGroup_[next[minPos]].deltaT_;

			Eigen::AngleAxisf rotater(base_.vel_.angular_ * (currT - lastT), Eigen::Vector3f(0, 0, 1));
			base_.translate(base_.origin_ + (base_.vel_.linear_ * (currT - lastT)));
			base_.rotate((Eigen::Vector3f)(rotater * base_.normal_), (Eigen::Vector3f)(rotater * base_.front_));
			for(int legIdx = 0; legIdx < 6; legIdx++) {
				base_.leg_[legIdx]->setPosition(base_.leg_[legIdx]->requestPosition(currT), currT - lastT);
//				std::cout << base_.leg_[legIdx]->requestPosition(currT) << '\n';
			}
//			std::cout << '\n';
			base_.writeSerial(uart_);
			//sleep currT - lastT
			usleep((currT - lastT) * 1000);
			lastT = currT;
		}
	}
}

void Hexapod::moveLinear(const Eigen::Vector3f& unitDisp, int stepT, int count) {
	int group = 1;
	const int sGroup[2][3] = {{0, 2, 4}, {1, 3, 5}};
	std::vector<int> sGroupVec[2];
	sGroupVec[0] = std::vector<int>(sGroup[0], sGroup[0] + 3); sGroupVec[1] = std::vector<int>(sGroup[1], sGroup[1] + 3);

	base_.stepGroup(unitDisp, stepT, sGroupVec[0], 40.f);
	base_.vel_.linear_ = unitDisp / stepT / 2;
	this->parseMovement();
	base_.resetMovementGroup(sGroupVec[0]);
//	base_.translate(base_.origin_ + unitMove);
//	base_.writeSerial(uart_);
//	usleep(500000);

	for(; count > 0; count--, group = (group + 1) % 2) {
		base_.stepGroup(unitDisp * 2, stepT, sGroupVec[group], 40.f);
		base_.vel_.linear_ = unitDisp / stepT;
		this->parseMovement();
		base_.resetMovementGroup(sGroupVec[group]);
//		base_.translate(base_.origin_ + unitMove);
//		base_.writeSerial(uart_);
//		usleep(500000);
	}

	base_.stepGroup(unitDisp, stepT, sGroupVec[group], 40.f);
	base_.vel_.linear_ = unitDisp / stepT / 2;
	this->parseMovement();
	base_.resetMovementGroup(sGroupVec[group]);
	base_.vel_.linear_ = Eigen::Vector3f::Zero();
}

void Hexapod::moveAngular(float unitAngularDisp, int stepT, int count) {
	int group = 1;
	const int sGroup[2][3] = {{0, 2, 4}, {1, 3, 5}};
	std::vector<int> sGroupVec[2];
	sGroupVec[0] = std::vector<int>(sGroup[0], sGroup[0] + 3); sGroupVec[1] = std::vector<int>(sGroup[1], sGroup[1] + 3);

	base_.turnGroup(unitAngularDisp, stepT, sGroupVec[0], 40.f);
	base_.vel_.angular_ = unitAngularDisp / stepT / 2;
	this->parseMovement();
	base_.resetMovementGroup(sGroupVec[0]);
//	base_.translate(base_.origin_ + unitMove);
//	base_.writeSerial(uart_);
//	usleep(500000);

	for(; count > 0; count--, group = (group + 1) % 2) {
		base_.turnGroup(unitAngularDisp * 2, stepT, sGroupVec[group], 40.f);
		base_.vel_.angular_ = unitAngularDisp / stepT;
		this->parseMovement();
		base_.resetMovementGroup(sGroupVec[group]);
//		base_.translate(base_.origin_ + unitMove);
//		base_.writeSerial(uart_);
//		usleep(500000);
	}

	base_.turnGroup(unitAngularDisp, stepT, sGroupVec[group], 40.f);
	base_.vel_.angular_ = unitAngularDisp / stepT / 2;
	this->parseMovement();
	base_.resetMovementGroup(sGroupVec[group]);
	base_.vel_.angular_ = 0;
}

void Hexapod::calibrate() {
	base_.calibrate(uart_);
}
