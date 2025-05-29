#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "math.hpp"

Quaternion IntegrateGyro(Quaternion current, Vector3 gyro, float dt);
Quaternion IntegrateAccel(Quaternion currrent, Vector3 accel, float a);

class EKF {
public:
	void Update(Vector3 gyro, Vector3 accel, Vector3 mag, float dt);
	Quaternion GetState() const;

	EKF();

public:
	Quaternion q        = {1, 0, 0, 0}; // Current State
	const float Q_STATE = EPSILON;      // State Noise         (Prediction Trust)
	const float Q_GYRO  = EPSILON;      // Gyroscope Noise     (Prediction Trust)
	const float R_ACCEL = EPSILON;      // Accelerometer Noise (Measurement Trust)
	const float R_MAG   = EPSILON;      // Magnetometer Noise  (Measurement Trust)
	Vector3 B           = {};           // Gyro Bias
	SquareMatrix<7> P   = {};           // State Covariance    (State Trust)
};

#endif /* KINEMATICS */
