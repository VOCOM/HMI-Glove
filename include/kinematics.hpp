#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "dataTypes.hpp"

Vector2 EulerFromAccel(Vector3 accelerometer);

void Body2World(Vector3 euler) {
	float& r      = euler.x;
	float& p      = euler.y;
	float& y      = euler.z;
	float R[3][3] = {
			{															cosf(p) * cosf(y),															 cosf(p) * cosf(y),          -sinf(p)},
			{										sinf(r) * sinf(p) * cosf(y), sinf(r) * sinf(p) * sinf(y) + cosf(r) * cosf(p), sinf(r) * cosf(p)},
			{cosf(r) * sinf(p) * cosf(y) + sinf(r) * sinf(y), cosf(r) * sinf(p) * sinf(y) - sinf(r) * cosf(y), cosf(r) * cosf(p)},
	};
}

#endif /* KINEMATICS */
