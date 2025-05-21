#include "kinematics.hpp"

#include <cmath>

Vector2 EulerFromAccel(Vector3 accelerometer) {
	float x2      = accelerometer.x * accelerometer.x;
	float y2      = accelerometer.y * accelerometer.y;
	float z2      = accelerometer.z * accelerometer.z;
	float gVector = sqrtf(x2 + y2 + z2);
	float roll    = atan2f(accelerometer.y, accelerometer.z);
	float pitch   = asinf(accelerometer.x / gVector);
	if (pitch == INFINITY) pitch = 0.0f;
	return Vector2{roll, pitch};
}
