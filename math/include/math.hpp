#ifndef MATH_HPP
#define MATH_HPP

#include "pico/double.h"
#include "pico/float.h"
#include "pico/stdlib.h"

#include "types.hpp"

#include "matrices.hpp"
#include "quaternions.hpp"
#include "vectors.hpp"

#include "filters.hpp"
#include "kinematics.hpp"

constexpr float G{9.80665};
constexpr float EPSILON{1e-6f};
constexpr float EARTH_ROTATION_DEG{4.1781e-3f};

constexpr float PI{3.1415927};
constexpr float PI_2{3.1415927 / 2};
constexpr float DEG2RAD{PI / 180.0};

constexpr Vector3 UnitX{1, 0, 0};
constexpr Vector3 UnitY{0, 1, 0};
constexpr Vector3 UnitZ{0, 0, 1};

struct Odometry {
	// Linear
	Vector3 Displacement;
	Vector3 Velocity;
	Vector3 Acceleration;

	// Angular
	Quaternion Orientation{1, 0, 0, 0};
};

#endif /* MATH */
