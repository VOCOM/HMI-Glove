#ifndef DATATYPES_HPP
#define DATATYPES_HPP

struct Vector2 {
	float x = 0;
	float y = 0;
};
struct Vector3 {
	float x = 0;
	float y = 0;
	float z = 0;

	Vector3 operator-(float rhs);
	Vector3 operator*(float rhs);
	Vector3 operator/(float rhs);
	Vector3 operator%(float rhs);
	Vector3& operator%=(float rhs);

	Vector3 operator-(Vector3& rhs);
	Vector3& operator+=(Vector3 rhs);
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

#endif /* DATATYPES */
