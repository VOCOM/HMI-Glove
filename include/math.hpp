#ifndef MATH_HPP
#define MATH_HPP

#include "pico/double.h"
#include "pico/float.h"

struct Vector2 {
	float x{};
	float y{};
};
struct Vector3 {
	float x{};
	float y{};
	float z{};

	Vector3 operator-(float rhs);
	Vector3 operator*(float rhs);
	Vector3 operator/(float rhs);
	Vector3 operator%(float rhs);

	Vector3& operator*=(float rhs);
	Vector3& operator/=(float rhs);
	Vector3& operator%=(float rhs);

	Vector3 operator-(Vector3& rhs);
	Vector3& operator+=(Vector3 rhs);

	void Round(int digits);
};

struct Matrix3x3 {
	float data[9]{};

	float& operator[](int index);

	Vector3 operator*(Vector3 rhs);

	static Matrix3x3 FromEuler(Vector3 angles);
};
struct Matrix4x4 {
	float data[16]{};
	float& operator[](int index);
};

struct Odometry {
	// Linear
	Vector3 Displacement;
	Vector3 Velocity;
	Vector3 Acceleration;

	// Angular
	Vector3 Euler;
	Vector3 EulerRate;
};

float EMA(float new_val, float old_val, float alpha);
Vector3 EMA(Vector3 new_val, Vector3 old_val, float alpha);

Vector2 EulerFromAccel(Vector3 accelerometer);

constexpr Vector3 G{0, 0, 9.81};
constexpr float PI{3.1415927};

#endif /* MATH */
