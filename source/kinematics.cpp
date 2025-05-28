#include "math.hpp"

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
	gyro -= R;
	Quaternion dq{0, gyro.x * 0.5f, gyro.y * 0.5f, gyro.z * 0.5f};
	Quaternion qDot = Derivative(q, dq);
	q += qDot;
	q.Normalize();

	// Update Error Covariance
	for (int i = 0; i < 4; i++) P[i][i] += Q.x;
	for (int i = 4; i < 7; i++) P[i][i] += Q.y;

	// Correction (Accel & Yaw)
	const Vector3 vAccel = accel.Cross(UnitZ);
	const Vector3 vMag   = mag.Cross(UnitY);
	const Vector3 vError = vAccel; //+ vMag;

	R += vError * (K.y * dt);
	dq   = Quaternion{0, vError.x * K.x, vError.y * K.x, vError.z * K.x};
	qDot = Derivative(q, dq);
	q += qDot * (0.5f * dt);
	q.Normalize();

	// Update Error Covariance
}

Quaternion EKF::GetState() const { return q; }

EKF::EKF() {
	for (int i = 0; i < 6; i++) P[i][i] = 0.01f;
}
