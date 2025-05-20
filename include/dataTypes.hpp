#ifndef DATATYPES_HPP
#define DATATYPES_HPP

struct Vector3 {
	float x = 0;
	float y = 0;
	float z = 0;

	Vector3 operator-(float rhs);
	Vector3 operator-(Vector3& rhs);
	Vector3 operator*(float rhs);
	Vector3& operator+=(Vector3 rhs);
};

struct Odometry {
	Vector3 Displacement;
	Vector3 Velocity;
	Vector3 Acceleration;
	Vector3 Orientation;
};

#endif /* DATATYPES */
