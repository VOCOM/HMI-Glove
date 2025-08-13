#ifndef MATRICES_HPP
#define MATRICES_HPP

#include "types.hpp"

struct DCM {
	float data[9]{};

	Vector3 ToEuler() const;
};

#endif /* MATRICES */
