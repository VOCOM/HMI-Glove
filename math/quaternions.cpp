#include "math.hpp"

Quaternion::Quaternion() {}
Quaternion::Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
Quaternion::Quaternion(const Vector3& axis, float angle) {
	float halfAngle = angle / 2.0f;
	float sinHalf   = sinf(halfAngle);

	w = cosf(halfAngle);
	x = axis.x * sinHalf;
	y = axis.y * sinHalf;
	z = axis.z * sinHalf;
}

Quaternion Quaternion::operator*(float scalar) const {
	return Quaternion{w * scalar, x * scalar, y * scalar, z * scalar};
}

Quaternion& Quaternion::operator*=(float scalar) {
	w *= scalar;
	x *= scalar;
	y *= scalar;
	z *= scalar;
	return *this;
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
	return Quaternion(
			w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
			w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
			w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
			w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w);
}

Quaternion& Quaternion::operator+=(const Quaternion& rhs) {
	w += rhs.w;
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}
Quaternion& Quaternion::operator*=(const Quaternion& rhs) {
	w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
	x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
	y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
	z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;
	return *this;
}

Quaternion& Quaternion::Normalize() {
	float m = sqrtf(w * w + x * x + y * y + z * z);
	if (m == INFINITY) return *this;
	w /= m;
	x /= m;
	y /= m;
	z /= m;
	return *this;
}

Vector3 Quaternion::ToVector3() {
	Vector3 newVec{};
	newVec.x = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
	newVec.y = asinf(2.0f * (w * y - z * x));
	newVec.z = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
	return newVec;
}

Quaternion FromVectors(const Vector3& lhs, const Vector3& rhs) {
	Vector3 u1 = Normalize(lhs), u2 = Normalize(rhs);
	Vector3 a = u1.Cross(u2);

	float m1 = u1.Magnitude(), m2 = u2.Magnitude();
	if (m1 < EPSILON) return Quaternion{1, 0, 0, 0};

	Quaternion q{};
	q.w = sqrtf(m1 * m1 * m2 * m2) + u1.Dot(u2);
	q.x = a.x;
	q.y = a.y;
	q.z = a.z;
	return q.Normalize();
}

Quaternion SLERP(Quaternion q1, Quaternion q2, float a) {
	float dot = q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q1.z * q2.z;

	if (dot > 0.9995f) {
		Quaternion r = {
				q1.w + a * (q2.w - q1.w),
				q1.x + a * (q2.x - q1.x),
				q1.y + a * (q2.y - q1.y),
				q1.z + a * (q2.z - q1.z)};
		return r.Normalize();
	}

	float theta_0     = acosf(dot);
	float theta       = theta_0 * a;
	float sin_theta   = sinf(theta);
	float sin_theta_0 = sinf(theta_0);

	float s0 = cosf(theta) - dot * sin_theta / sin_theta_0;
	float s1 = sin_theta / sin_theta_0;

	Quaternion r = {
			s0 * q1.w + s1 * q2.w,
			s0 * q1.x + s1 * q2.x,
			s0 * q1.y + s1 * q2.y,
			s0 * q1.z + s1 * q2.z};
	return r.Normalize();
}
