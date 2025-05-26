#include "kinematics.hpp"
#include "math.hpp"

Quaternion IntegrateGyro(Quaternion current, Vector3 gyro, float dt) {
	float m = gyro.Magnitude();
	if (m < EPSILON) return current;

	gyro.Normalize();
	float theta = m * dt;
	float s2t   = sinf(theta / 2.0f);
	float c2t   = cosf(theta / 2.0f);

	Quaternion dq(c2t, gyro.x * s2t, gyro.y * s2t, gyro.z * s2t);
	current *= dq.Normalize();
	return current.Normalize();
}

Quaternion IntegrateAccel(Quaternion current, Vector3 accel, float a) {
	Vector3 orientation = current.ToVector3();
	Vector3 vAccel      = FromVectors(accel, UnitZ).ToVector3();
	vAccel.z            = orientation.z;

	// Fused Orientation Quaternion
	return SLERP(current, vAccel.ToQuaternion(), a);
}
