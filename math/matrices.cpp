#include "matrices.hpp"
#include "math.hpp"

Vector3 DCM::ToEuler() const {
	Vector3 euler;

	euler.y = -asinf(data[6]);

	if (std::abs(data[6]) < 0.99999f) {
		euler.x = atan2f(data[7], data[8]);
		euler.z = atan2f(data[3], data[0]);
	} else {
		euler.x = 0;
		euler.z = atan2f(-data[1], data[4]);
	}

	return euler;
}
