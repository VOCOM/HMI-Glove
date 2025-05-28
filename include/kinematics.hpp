#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "types.hpp"

Quaternion IntegrateGyro(Quaternion current, Vector3 gyro, float dt);
Quaternion IntegrateAccel(Quaternion currrent, Vector3 accel, float a);

class EKF {
public:
	void Update(Vector3 gyro, Vector3 accel, Vector3 mag, float dt);
	Quaternion GetState() const;

	EKF();

private:
	Quaternion q{1, 0, 0, 0};

	Vector2 K{1.0f, 1.0f};   // Kalman Gain
	SquareMatrix<7> P;       // Error Covariance Matrix
	Vector2 Q{1e-5f, 1e-6f}; // Process Noise Matrix
	Vector3 R;               // Measurement Noise Matrix
};

#endif /* KINEMATICS */
