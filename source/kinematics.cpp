#include "kinematics.hpp"

Quaternion IntegrateGyro(Quaternion current, Vector3 gyro, float dt) {
	float m = gyro.Magnitude();
	if (m < EPSILON) return current;

	gyro.Normalize();
	float theta = m * dt;
	float s2t   = sinf(theta / 2.0f);
	float c2t   = cosf(theta / 2.0f);

	Quaternion dq(c2t, gyro.x * s2t, gyro.y * s2t, gyro.z * s2t);
	current *= dq.Normalize();
	return current.Normalize();
}
Quaternion IntegrateAccel(Quaternion current, Vector3 accel, float a) {
	Vector3 orientation = current.ToVector3();
	Vector3 vAccel      = FromVectors(accel, UnitZ).ToVector3();
	vAccel.z            = orientation.z;

	// Fused Orientation Quaternion
	return SLERP(current, vAccel.ToQuaternion(), a);
}

void EKF::Update(Vector3 gyro, Vector3 accel, Vector3 mag, float dt) {
	// State Prediction (Gyro only)
	gyro -= B;
	q += Derivative(q, Quaternion{0, gyro.x, gyro.y, gyro.z}) * dt;
	q.Normalize();

	// Update State Covariance (Lose Trust)
	for (int i = 0; i < 4; i++) P[i][i] += Q_STATE;
	for (int i = 4; i < 7; i++) P[i][i] += Q_GYRO;

	if (accel.Magnitude() < EPSILON) return;
	accel.Normalize();
	mag.Normalize();

	Vector3 gError = accel - UnitZ.Rotate(q);
	Vector3 mError = mag - UnitY.Rotate(q);
	float y[6]{gError.x, gError.y, gError.z, mError.x, mError.y, mError.z};

	// Measurement Jacobian (Mapping matrix)
	float w2 = 2 * q.w, x2 = 2 * q.x, y2 = 2 * q.y, z2 = 2 * q.z;
	float H[6][7]{
			{ y2, -z2,  w2, -x2, 0, 0, 0},
			{-x2, -w2, -z2, -y2, 0, 0, 0},
			{-w2,  x2,  y2, -z2, 0, 0, 0},
			{-z2,  y2,  x2, -w2, 0, 0, 0},
			{ w2,  x2,  y2,  z2, 0, 0, 0},
			{ x2, -w2,  z2, -y2, 0, 0, 0}
  };

	// Error Covariance
	float S[6][6]{};
	for (int i = 0; i < 6; ++i)
		for (int j = 0; j < 6; ++j)
			for (int k = 0; k < 7; ++k)
				S[i][j] += H[i][k] * P[k][k] * H[j][k];

	// Add Measurement Noise
	for (int i = 0; i < 6; ++i)
		S[i][i] += i < 3 ? R_ACCEL : R_MAG;

	// Kalman Gain
	float K[7][6] = {};
	for (int i = 0; i < 7; ++i)
		for (int j = 0; j < 6; ++j)
			if (S[j][j] > 0.0f) K[i][j] = P[i][i] * H[j][i] / S[j][j];

	// Correct State Prediction
	for (int i = 0; i < 7; ++i) {
		float dx = 0;
		for (int j = 0; j < 6; ++j)
			dx += K[i][j] * y[j];

		if (i == 0) q.w += dx;
		if (i == 1) q.x += dx;
		if (i == 2) q.y += dx;
		if (i == 3) q.z += dx;
		if (i == 4) B.x += dx;
		if (i == 5) B.y += dx;
		if (i == 6) B.z += dx;
	}
	q.Normalize();

	// Update State Covariance (Gain Trust)
	float KH[7][7] = {};
	for (int i = 0; i < 7; ++i)
		for (int j = 0; j < 7; ++j)
			for (int k = 0; k < 6; ++k)
				KH[i][j] += K[i][k] * H[k][j];

	float I_KH[7][7] = {};
	for (int i = 0; i < 7; ++i)
		for (int j = 0; j < 7; ++j)
			I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];

	float P_new[7][7] = {};
	for (int i = 0; i < 7; ++i)
		for (int j = 0; j < 7; ++j)
			for (int k = 0; k < 7; ++k)
				P_new[i][j] += I_KH[i][k] * P[k][j];

	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 7; j++)
			P[i][j] = P_new[i][j];
}

Quaternion EKF::GetState() const { return q; }

EKF::EKF() {
	for (int i = 0; i < 6; i++) P[i][i] = 0.01f;
}
