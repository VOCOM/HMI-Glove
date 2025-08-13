#ifndef QUATERNIONS_HPP
#define QUATERNIONS_HPP

#include "types.hpp"

struct Quaternion {
	float w{};
	float x{};
	float y{};
	float z{};

	Quaternion();
	Quaternion(float w, float x, float y, float z);
	/// @brief Vector Initialization
	/// @param x
	/// @param y
	/// @param z
	Quaternion(float x, float y, float z);
	Quaternion(const Vector3& axis, float angle);

	Quaternion operator*(float scalar) const;

	Quaternion& operator*=(float rhs);

	Quaternion operator*(const Quaternion& rhs) const;

	Quaternion& operator+=(const Quaternion& rhs);
	Quaternion& operator*=(const Quaternion& rhs);

	Quaternion& Normalize();

	Quaternion Conjugate();

	Vector3 ToVector3() const;
};

Quaternion FromGyro(Vector3 gyro, float dt);

Quaternion FromVectors(const Vector3& lhs, const Vector3& rhs);
Quaternion SLERP(Quaternion q1, Quaternion q2, float a);
Quaternion Derivative(Quaternion q1, Quaternion q2);

#endif /* QUATERNIONS */
