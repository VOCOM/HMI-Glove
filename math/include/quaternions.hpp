#ifndef QUATERNIONS_HPP
#define QUATERNIONS_HPP

#include "types.hpp"

struct Quaternion {
	float w{}, x{}, y{}, z{};

	Quaternion();
	Quaternion(float w, float x, float y, float z);
	Quaternion(const Vector3& axis, float angle);

	Quaternion operator*(float scalar) const;

	Quaternion& operator*=(float rhs);

	Quaternion operator*(const Quaternion& rhs) const;

	Quaternion& operator+=(const Quaternion& rhs);
	Quaternion& operator*=(const Quaternion& rhs);

	Quaternion& Normalize();

	Vector3 ToVector3();
};

Quaternion FromVectors(const Vector3& lhs, const Vector3& rhs);
Quaternion SLERP(Quaternion q1, Quaternion q2, float a);

#endif /* QUATERNIONS */
