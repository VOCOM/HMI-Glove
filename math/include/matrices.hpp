#ifndef MATRICES_HPP
#define MATRICES_HPP

#include "types.hpp"

template <int N>
struct SquareMatrix {
	float data[N][N]{};

	float* operator[](int index) {
		if (index > N) panic("Index out of bounds!\n");
		return data[index];
	}
};

struct Matrix3x3 {
	float data[9]{};

	float& operator[](int index);

	Vector3 operator*(Vector3 rhs);

	static Matrix3x3 FromEuler(Vector3 angles);
};

#endif /* MATRICES */
