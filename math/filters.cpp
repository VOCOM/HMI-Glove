#include "math.hpp"

float CF(float lhs, float rhs, float alpha) {
	if (alpha > 1.0f) alpha = 1.0f;
	return alpha * lhs + (1.0f - alpha) * rhs;
}

Vector3 CF(Vector3 lhs, Vector3 rhs, float alpha) {
	if (alpha > 1) alpha = 1.0f;
	Vector3 ema_val;
	ema_val.x = CF(lhs.x, rhs.x, alpha);
	ema_val.y = CF(lhs.y, rhs.y, alpha);
	ema_val.z = CF(lhs.z, rhs.z, alpha);
	return ema_val;
}

void KF::Predict(Quaternion& q, Vector3 gyro, Vector3 accel, float dt) {
	// Prediction
	Quaternion qDot{
			0,
			gyro.x - gyroBias.x,
			gyro.y - gyroBias.y,
			gyro.z - gyroBias.z};
	qDot = q * qDot * 0.5f * dt;

	// Update Orientation
	q += qDot;
	q.Normalize();

	// Predict Covariance
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			P[i][j] += Q_proc * dt;

	for (int i = 4; i < 7; i++)
		P[i][i] += Q_bias * dt;

	// Accelerometer Correction
	// Predicted gravity in body frame
	Vector3 gPred = {
			2 * (q.x * q.z - q.w * q.y),
			2 * (q.w * q.x + q.y * q.z),
			q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
	Vector3 error = accel - gPred;

	// Kalman gain (simplified)
	float K[7][3];
	for (int i = 0; i < 7; i++)
		for (int j = 0; j < 3; j++)
			K[i][j] = P[i][j] / (P[j][j] + R_accel);

	// Update Orientation
	for (int i = 0; i < 4; i++) {
		q.w += K[i][0] * error.x;
		q.x += K[i][1] * error.y;
		q.y += K[i][2] * error.z;
	}
	q.Normalize();

	gyroBias.x += K[4][0] * error.x;
	gyroBias.y += K[5][0] * error.x;
	gyroBias.z += K[6][0] * error.x;
}
