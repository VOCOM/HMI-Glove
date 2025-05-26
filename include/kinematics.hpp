#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "types.hpp"

Quaternion IntegrateGyro(Quaternion current, Vector3 gyro, float dt);
Quaternion IntegrateAccel(Quaternion currrent, Vector3 accel, float a);

#endif /* KINEMATICS */
