#ifndef VECTORS_HPP
#define VECTORS_HPP

#include "types.hpp"

struct Vector2 {
	float x{}, y{};
};

struct Vector3 {
	float x{}, y{}, z{};

	// Scalar arithmetic

	Vector3 operator-() const;

	Vector3 operator+(float rhs) const;
	Vector3 operator-(float rhs) const;
	Vector3 operator*(float rhs) const;
	Vector3 operator/(float rhs) const;
	Vector3 operator%(float rhs) const;

	Vector3& operator-=(float rhs);
	Vector3& operator*=(float rhs);
	Vector3& operator/=(float rhs);
	Vector3& operator%=(float rhs);

	// Vector arithmetic

	Vector3 operator-(Vector3& rhs) const;

	Vector3 operator+(Vector3 rhs) const;

	Vector3& operator+=(Vector3 rhs);
	Vector3& operator-=(Vector3 rhs);

	void Round(int digits);
	Vector3& Normalize();
	float Magnitude() const;

	float Dot(const Vector3& rhs) const;
	Vector3 Cross(const Vector3& rhs) const;

	Quaternion ToQuaternion();
};

Vector3 Normalize(const Vector3& vec);

#endif /* VECTORS */
