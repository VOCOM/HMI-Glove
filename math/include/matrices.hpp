#ifndef MATRICES_HPP
#define MATRICES_HPP

#include "types.hpp"

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

#endif /* MATRICES */
