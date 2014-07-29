/*
 * uscengine.cpp
 *
 *  Created on: Jul 25, 2014
 *      Author: core
 */

#include <uscengine.h>

/* These should be private */
struct Plane;

typedef struct {
	int number, minPW, maxPW, curPW, actTime;
	float angle;
	bool changed;
} Servo;

typedef struct Movement {
	Vector3f position;
	int dT;
} Movement;

typedef struct {
	Servo servo[3];
	Vector3f postion, origin;
	struct Plane* refPlane;
	SideType side;
	Movement mstack[100];
	int mtop;
} Leg;

typedef struct Plane {
	Leg leg[6];
	Velocity vel;
	Vector3f origin, normal;
	float dire;
} Plane;

Plane plane;
float leg1Len, leg2Len, len3, len4;
const Vector3f zAxis = {0, 0, 1};
const size_t smax = 10000;

/* Declaration */
Vector3f getProjection(const Plane* pp, const Vector3f* point);

/* v3f operation */
float v3fDot(const Vector3f* va, const Vector3f* vb) {
	return( va->x * vb->x +
		    va->y * vb->y +
		    va->z * vb->z );
}

Vector3f v3fMul(const Vector3f* v, float num) {
	Vector3f result = *v;
	result.x *= num;
	result.y *= num;
	result.z *= num;
	return result;
}

Vector3f v3fDiv(const Vector3f* v, float num) {
	Vector3f result = *v;
	result.x /= num;
	result.y /= num;
	result.z /= num;
	return result;
}

Vector3f v3fSub(const Vector3f* va, const Vector3f* vb) {
	Vector3f result;
	result.x = va->x - vb->x;
	result.y = va->y - vb->y;
	result.z = va->z - vb->z;
	return result;
}

Vector3f v3fAdd(const Vector3f* va, const Vector3f* vb) {
	Vector3f result;
	result.x = va->x + vb->x;
	result.y = va->y + vb->y;
	result.z = va->z + vb->z;
	return result;
}

Vector3f v3fCross(const Vector3f* va, const Vector3f* vb) {
	Vector3f result;
	// ax ay az
	// bx by bz
	result.x = va->y * vb->z - va->z * vb->y;
	result.y = va->z * vb->x - va->x * vb->z;
	result.z = va->x * vb->y - va->y * vb->x;
	return result;
}

float v3fNorm(const Vector3f* v) {
	return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}

float v3fSquaredNorm(const Vector3f* v) {
	return (v->x * v->x + v->y * v->y + v->z * v->z);
}

Vector3f v3fAARot(const Vector3f* v, const Vector3f* axis, float angle) {

	Vector3f lv = *v,
			 t1 = v3fMul(&lv, cos(angle)),
			 w = v3fDiv(axis, v3fNorm(axis)),
			 t2 = v3fCross(&w, &lv),
			 t3 = w;
	t2 = v3fMul(&t2, sin(angle));
	t1 = v3fAdd(&t1, &t2);
	t3 = v3fMul(&t3, v3fDot(&w, &lv) * (1 - cos(angle)));
	t1 = v3fAdd(&t1, &t3);
	return t1;
}


/* Servo */
void setAngle(Servo* ps, float angle) {
	int newPW;
	ps->angle = angle;
	newPW = ps->minPW + (ps->maxPW - ps->minPW) * angle / PI;
	if(newPW != ps->curPW) {
		ps->changed = true;
		ps->curPW = newPW;
	}
}

/* Leg */
void addMovement(Leg* pl, const Vector3f* pos, int dT) {
	pl->mstack[pl->mtop].position = *pos;
	pl->mstack[pl->mtop].dT = dT;
	pl->mtop++;
}

void resetMovement(Leg* pl) {
	pl->mstack[0].position = pl->postion;
	pl->mstack[0].dT = 0;
	pl->mtop = 1;
}

Vector3f requestPosition(Leg* pl, int time) {
	Vector3f result = pl->postion;
	if(pl->mtop == 0)
		return result;
	int idx = 0, sumT = 0;
	Vector3f lastPosition = pl->mstack[0].position;
	for(idx++; idx < pl->mtop && sumT + pl->mstack[pl->mtop].dT < time; sumT += pl->mstack[idx].dT, lastPosition = pl->mstack[idx].position, idx++);
	if(idx == pl->mtop)
		return lastPosition;
	else {
		result = v3fSub(&pl->mstack[idx].position, &lastPosition);
		result = v3fMul(&result, (float)(time - sumT) / pl->mstack[idx].dT);
		result = v3fAdd(&result, &lastPosition);
		return result;
	}
}
	
void setPosition(Leg* pl, const Vector3f* pos, int time) {
	pl->postion = *pos;
	Vector3f pp = getProjection(pl->refPlane, pos),
			 ol = pl->origin,
			 t1 = v3fAdd(&pl->origin, &pl->refPlane->origin),
			 lp = v3fSub(&pp, &t1),
			 olp = v3fCross(&ol, &lp);
	float alpha = asin(len3 / v3fNorm(&lp)),
		  beta  = acos(v3fDot(&ol, &lp) / (v3fNorm(&ol) * v3fNorm(&lp)));
	if(v3fDot(&olp, &pl->refPlane->normal) < 0)
		beta = -beta;
	if(pl->side == left)
		setAngle(&pl->servo[2], beta + alpha);
	else
		setAngle(&pl->servo[2], beta - alpha);
	Vector3f ob = v3fCross(&pl->origin, &pl->refPlane->normal),
			 obOffset,
			 bb,
			 bd;
	obOffset = v3fAdd(&pl->refPlane->origin, &pl->origin);
			   t1 = v3fMul(&pl->refPlane->normal, -len4 / v3fNorm(&pl->refPlane->normal));
			   obOffset = v3fAdd(&obOffset, &t1);
	float lbd, lbd2, theta, delta;
	const float lbc2 = leg2Len * leg2Len,
		        lcd2 = leg1Len * leg1Len,
				lbc = leg2Len,
				lcd = leg1Len;
	ob = v3fAARot(&ob, &pl->refPlane->normal, pl->servo[2].angle);
	if(pl->side == left) {
		ob = v3fMul(&ob, len3 / v3fNorm(&ob));
	} else {
		ob = v3fMul(&ob, -len3 / v3fNorm(&ob));
	}
	bb = v3fAdd(&ob, &obOffset);
	bd = v3fSub(&pl->postion, &bb);
	lbd = v3fNorm(&bd);
	lbd2 = lbd * lbd;
	setAngle(&pl->servo[0], acos((lbc2 + lcd2 - lbd2) / (2 * lbc * lcd)));
	theta = acos((lbc2 + lbd2 - lcd2) / (2 * lbc * lbd));
	delta = acos(v3fDot(&bd, &pl->refPlane->normal) / v3fNorm(&bd));
	setAngle(&pl->servo[1], delta - theta);
	pl->servo[0].actTime = pl->servo[1].actTime = pl->servo[2].actTime = time;
	if(time < 50)
		time = 50;
}

void setOrigin(Leg* pl, const Vector3f* newOrigin, int time) {
	pl->origin = *newOrigin;
	setPosition(pl, &pl->postion, time);
}

void step(Leg* pl, const Vector3f* unitDisp, int totalT, float height) {
	Vector3f t1, t2, t3;
	resetMovement(pl);
	t1 = v3fMul(&pl->refPlane->normal, height);
	t2 = v3fAdd(&pl->postion, &t1);
	addMovement(pl, &t2, totalT / 3);
	t3 = v3fAdd(&pl->postion, unitDisp);
	t2 = v3fAdd(&t3, &t1);
	addMovement(pl, &t2, totalT / 3);
	addMovement(pl, &t3, totalT / 3);
}

void turn(Leg* pl, float unitADisp, int totalT, float height) {
	Vector3f t1, t2,  newRelPos, newPos;
	resetMovement(pl);
	t1 = v3fSub(&pl->postion, &pl->refPlane->origin);
	t2.x = t2.y = 0; t2.z = 1;
	newRelPos = v3fAARot(&t1, &t2, unitADisp);
	newPos = v3fAdd(&newRelPos, &pl->refPlane->origin);
	t1 = v3fMul(&pl->refPlane->normal, height);
	t2 = v3fAdd(&t1, &pl->postion);
	addMovement(pl, &t2, totalT / 3);
	t2 = v3fAdd(&t1, &newPos);
	addMovement(pl, &t2, totalT / 3);
	addMovement(pl, &newPos, totalT / 3);
}


/* Plane */
const int cGroup[2][3] = { {0, 2, 4}, {1, 3, 5} };

Vector3f getProjection(const Plane* pp, const Vector3f* point) {
	Vector3f op = v3fSub(point, &pp->origin), result, t1;
	float lProj = v3fDot(&op, &pp->normal);
	t1 = v3fMul(&pp->normal, lProj);
	result = v3fSub(point, &t1);
	return result;
}

void stepGroup(Plane* pp, const Vector3f* unitDisp, int stepT, float height, int group) {
	for(int i = 0; i < 3; i++)
		step(&pp->leg[cGroup[group][i]], unitDisp, stepT, height);
}

void turnGroup(Plane* pp, float unitADisp, int stepT, float height, int group) {
	for(int i = 0; i < 3; i++)
		turn(&pp->leg[cGroup[group][i]], unitADisp, stepT, height);
}

void resetMovementGroup(Plane* pp, int group) {
	for(int i = 0; i < 3; i++)
		resetMovement(&pp->leg[cGroup[group][i]]);
}

/* Robot Level */
size_t minElemIdx(size_t* p, size_t len) {
	size_t ans = 0, minv = smax;
	for(size_t i = 0; i < len; i++)
		if(minv > p[i]) {
			minv = p[i];
			ans = i;
		}
	return ans;
}

void translate(const Vector3f* newOrigin, int time) {
	plane.origin = *newOrigin;
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		setPosition(&plane.leg[legIdx], &plane.leg[legIdx].postion, time);
	}
}

void rotate(float angle, int time) {
	int legIdx;
	for(legIdx = 0; legIdx < 6; legIdx++) {
		Vector3f t, t2;
		t2.x = t2.y = 0; t2.z = 1;
		t = v3fAARot(&plane.leg[legIdx].origin, &t2, angle);
		setOrigin(&plane.leg[legIdx], &t, time);
	}
	plane.dire += angle;
}

Vector3f tfVector(const Vector3f* v) {
	Vector3f result;
	float r = remainderf(plane.dire, PI);
	result = v3fAARot(v, &zAxis, r);
	return result;
}
		
void syncServoData() {
	for(int legIdx = 0; legIdx < 6; legIdx++)
		for(int servoIdx = 0; servoIdx < 3; servoIdx++) {
			Servo* ps = &plane.leg[legIdx].servo[servoIdx];
			if(ps->changed) {
				printf("#%dP%dT%d", ps->number, ps->curPW, ps->actTime);
				ps->changed = false;
			}
		}
	printf("\n");
}

void parseMovement() {
	size_t nextT[6] = {0}, next[6] = {1, 1, 1, 1, 1, 1};
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		if(plane.leg[legIdx].mtop == 1)
			nextT[legIdx] = smax;
		else
			nextT[legIdx] = plane.leg[legIdx].mstack[1].dT;
	}
	size_t lastT = 0, currT = 0;
	for(int minPos = minElemIdx(nextT, 6); nextT[minPos] != smax; minPos = minElemIdx(nextT, 6)) {
		Leg* minLeg = &plane.leg[minPos];
		if(nextT[minPos] <= currT) {
			if((++next[minPos]) == (size_t)minLeg->mtop)
				nextT[minPos] = smax;
			else
				nextT[minPos] += minLeg->mstack[next[minPos]].dT;
			continue;
		} else {
			currT = nextT[minPos];
			if((++next[minPos]) == (size_t)minLeg->mtop)
				nextT[minPos] = smax;
			else
				nextT[minPos] += minLeg->mstack[next[minPos]].dT;
			// translate
			Vector3f t, t2;
			t2 = v3fMul(&plane.vel.linear, currT - lastT);
			t = v3fAdd(&plane.origin, &t2);
			translate(&t, currT - lastT);
			rotate(plane.vel.angular * (currT - lastT), currT - lastT);
			for(int legIdx = 0; legIdx < 6; legIdx++) {
				Vector3f t = requestPosition(&plane.leg[legIdx], currT);
				setPosition(&plane.leg[legIdx], &t, currT - lastT);
			}
			syncServoData();
			//usleep;
			lastT = currT;
		}
	}
}

/* public */
void walk(float x, float y, int count, int time) {
	if(count <= 0)
		return;
	int group = 1;
	Vector3f unitDisp = {x, y, 0}, unitDisp2;
	unitDisp = tfVector(&unitDisp);

	stepGroup(&plane, &unitDisp, time, 20.f, 0);
	plane.vel.linear = v3fDiv(&unitDisp, time * 2);
	parseMovement();
	resetMovementGroup(&plane, 0);
	
	unitDisp2 = v3fMul(&unitDisp, 2);
	plane.vel.linear = v3fDiv(&unitDisp, time);
	for(; count > 1; count--, group = !group) {
		stepGroup(&plane, &unitDisp2, time, 20.f, group);
		parseMovement();
		resetMovementGroup(&plane, group);
	}

	stepGroup(&plane, &unitDisp, time, 20.f, group);
	plane.vel.linear = v3fDiv(&unitDisp, time * 2);
	parseMovement();
	resetMovementGroup(&plane, group);
	plane.vel.linear.x = plane.vel.linear.y = plane.vel.linear.z = 0;
}

void roll(float angle, int count, int time) {
	if(count <= 0)
		return;
	int group = 1;

	turnGroup(&plane, angle, time, 20.f, 0);
	plane.vel.angular = angle / time / 2;
	parseMovement();
	resetMovementGroup(&plane, 0);

	plane.vel.angular = angle / time;
	for(; count > 1; count--, group = !group) {
		turnGroup(&plane, angle * 2, time, 20.f, group);
		parseMovement();
		resetMovementGroup(&plane, group);
	}
	plane.vel.angular = 0;

	turnGroup(&plane, angle, time, 20.f, group);
	plane.vel.angular = angle / time / 2;
	parseMovement();
	resetMovementGroup(&plane, group);
	plane.vel.angular = 0;
}

void initialize(const char *paramFilePath) {
	FILE *fp = fopen(paramFilePath, "r");
	leg1Len = 140.801278;
	leg2Len = 86.0;
	len3 = 8.0;
	len4 = 29.0;
	plane.leg[0].origin.x = 75.0;
	plane.leg[0].origin.y = 40.0;
	plane.leg[0].origin.z = 0;
	plane.leg[1].origin.x = 0;
    plane.leg[1].origin.y = 67.8388;
    plane.leg[1].origin.z = 0;
	plane.leg[2].origin.x = -75;
    plane.leg[2].origin.y = 40.0;
    plane.leg[2].origin.z = 0;
	plane.leg[3].origin.x = 75;
    plane.leg[3].origin.y = -40.0;
    plane.leg[3].origin.z = 0;
	plane.leg[4].origin.x = 0;
    plane.leg[4].origin.y = -67.8388;
    plane.leg[4].origin.z = 0;
	plane.leg[5].origin.x = -75;
    plane.leg[5].origin.y = -40;
    plane.leg[5].origin.z = 0;
	plane.leg[0].postion.x = 150.8824;
	plane.leg[0].postion.y = 80.4706;
	plane.leg[0].postion.z = 0;
	plane.leg[1].postion.x = 0;
    plane.leg[1].postion.y = 153.8388;
    plane.leg[1].postion.z = 0;
	plane.leg[2].postion.x = -158.8824;
    plane.leg[2].postion.y = 80.4706;
    plane.leg[2].postion.z = 0;
	plane.leg[3].postion.x = 150.8824;
    plane.leg[3].postion.y = -80.4706;
    plane.leg[3].postion.z = 0;
	plane.leg[4].postion.x = 0;
    plane.leg[4].postion.y = -153.8388;
    plane.leg[4].postion.z = 0;
	plane.leg[5].postion.x = -158.8824;
    plane.leg[5].postion.y = -80.4706;
    plane.leg[5].postion.z = 0;
	for(int legIdx = 0; legIdx < 6; legIdx++) {
		plane.leg[legIdx].refPlane = &plane;
		if(legIdx > 2)
			plane.leg[legIdx].side = right;
		else
			plane.leg[legIdx].side = left;
		resetMovement(&plane.leg[legIdx]);
		for(int servoIdx = 0; servoIdx < 3; servoIdx++) {
			Servo *ps = &plane.leg[legIdx].servo[servoIdx];
			fscanf(fp, "%d %d %d", &ps->number, &ps->minPW, &ps->maxPW);
			ps->curPW = 1500;
			ps->angle = 0;
			ps->changed = false;
		}
	}
	fclose(fp);
	Vector3f zeros = {0, 0, 0};
	plane.origin.x = plane.origin.y = 0; plane.origin.z = 0;
	plane.vel.linear = zeros;
	plane.vel.angular = 0;
	plane.dire = 0;
	plane.normal = zAxis;
	Vector3f init = {0, 0, 120};
	translate(&init, 3000);
	syncServoData();
}

