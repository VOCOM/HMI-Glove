#include "kinematics.hpp"

void UpdatePrediction(Quaternion& q, const Vector3& gyro, float dt) {
	float half_dt = 0.5f * dt;
	Quaternion dq = {1, gyro.x * half_dt, gyro.y * half_dt, gyro.z * half_dt};
	q *= dq;
	q.Normalize();
}
void UpdateModel(Quaternion& m, const Vector3& accel, const Vector3& mag) {
	// Normalize
	Vector3 aVec = Normalize(accel);
	Vector3 mVec = Normalize(mag);

	float roll  = atan2f(aVec.y, aVec.z);
	float pitch = atan2f(-aVec.x, sqrtf(aVec.y * aVec.y + aVec.z * aVec.z));

	float pitch_cos = cosf(pitch);
	float pitch_sin = sinf(pitch);
	float roll_cos  = cosf(roll);
	float roll_sin  = sinf(roll);

	// Rotate Magnetometer
	Vector2 mLeveled;
	mLeveled.x = mVec.x * pitch_cos + mVec.z * pitch_sin;
	mLeveled.y = mVec.x * roll_sin * pitch_sin + mVec.y * roll_cos - mVec.z * roll_sin * pitch_cos;
	float yaw  = atan2f(mLeveled.x, mLeveled.y);

	m = Vector3{roll, pitch, yaw}.ToQuaternion();
}

void EKF::Update(Vector3 gyro, Vector3 accel, Vector3 mag, float dt) {
	// State Prediction (Gyro only)
	gyro -= B;
	q += Derivative(q, Quaternion{0, gyro.x, gyro.y, gyro.z}) * dt;
	q.Normalize();

	// Update State Covariance (Lose Trust)
	// for (int i = 0; i < 4; i++) P[i][i] += Q_STATE;
	// for (int i = 4; i < 7; i++) P[i][i] += Q_GYRO;

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
			for (int k = 0; k < 7; ++k);
	// S[i][j] += H[i][k] * P[k][k] * H[j][k];

	// Add Measurement Noise
	for (int i = 0; i < 6; ++i)
		S[i][i] += i < 3 ? R_ACCEL : R_MAG;

	// Kalman Gain
	float K[7][6] = {};
	for (int i = 0; i < 7; ++i)
		for (int j = 0; j < 6; ++j);
	// if (S[j][j] > 0.0f) K[i][j] = P[i][i] * H[j][i] / S[j][j];

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
			for (int k = 0; k < 7; ++k);
	// P_new[i][j] += I_KH[i][k] * P[k][j];

	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 7; j++);
	// P[i][j] = P_new[i][j];
}

Quaternion EKF::GetState() const { return q; }

EKF::EKF() {
	// for (int i = 0; i < 6; i++) P[i][i] = 0.01f;
}
