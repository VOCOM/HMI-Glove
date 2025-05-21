#include "math.hpp"

Vector3 Vector3::operator-(float rhs) {
	return Vector3{this->x - rhs, this->y - rhs, this->z - rhs};
}
Vector3 Vector3::operator*(float rhs) {
	return Vector3{this->x * rhs, this->y * rhs, this->z * rhs};
}
Vector3 Vector3::operator/(float rhs) {
	return Vector3{this->x / rhs, this->y / rhs, this->z / rhs};
}
Vector3 Vector3::operator%(float rhs) {
	return Vector3{(float)fmod(this->x, rhs), (float)fmod(this->x, rhs), (float)fmod(this->x, rhs)};
}

Vector3& Vector3::operator*=(float rhs) {
	this->x *= rhs;
	this->y *= rhs;
	this->z *= rhs;
	return *this;
}
Vector3& Vector3::operator/=(float rhs) {
	this->x /= rhs;
	this->y /= rhs;
	this->z /= rhs;
	return *this;
}
Vector3& Vector3::operator%=(float rhs) {
	this->x = (float)fmod(this->x, rhs);
	this->y = (float)fmod(this->y, rhs);
	this->z = (float)fmod(this->z, rhs);
	return *this;
}

Vector3 Vector3::operator-(Vector3& rhs) {
	return Vector3{this->x - rhs.x, this->y - rhs.y, this->z - rhs.z};
}

Vector3& Vector3::operator+=(Vector3 rhs) {
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;
	return *this;
}

void Vector3::Round(int digits) {
	for (int i = 0; i < digits; i++) *this *= 10;
	this->x = round(this->x);
	this->y = round(this->y);
	this->z = round(this->z);
	for (int i = 0; i < digits; i++) *this /= 10;
}

float& Matrix3x3::operator[](int index) {
	return data[index];
}
Vector3 Matrix3x3::operator*(Vector3 rhs) {
	Vector3 newVec{};
	newVec.x = data[0] * rhs.x + data[1] * rhs.y + data[2] * rhs.z;
	newVec.y = data[3] * rhs.x + data[4] * rhs.y + data[5] * rhs.z;
	newVec.z = data[6] * rhs.x + data[7] * rhs.y + data[8] * rhs.z;
	return newVec;
}
Matrix3x3 Matrix3x3::FromEuler(Vector3 angles) {
	float& r = angles.x;
	float& p = angles.y;
	float& y = angles.z;
	Matrix3x3 newMat{};
	newMat[0] = cosf(p) * cosf(y);
	newMat[1] = sinf(r) * sinf(p) * cosf(y) - cosf(r) * sinf(y);
	newMat[2] = cosf(r) * sinf(p) * cosf(y) + sinf(r) * sinf(y);
	newMat[3] = cosf(p) * sinf(y);
	newMat[4] = sinf(r) * sinf(p) * sinf(y) + cosf(r) * cosf(y);
	newMat[5] = cosf(r) * sinf(p) * sinf(y) - sinf(r) * cosf(y);
	newMat[6] = -sinf(p);
	newMat[7] = sinf(r) * cosf(p);
	newMat[8] = cosf(r) * cosf(p);
	return newMat;
}

float& Matrix4x4::operator[](int index) {
	return data[index];
}

float EMA(float new_val, float old_val, float alpha) {
	if (alpha > 1) alpha = 1.0f;
	return (new_val * alpha) + (old_val * (1 - alpha));
}
Vector3 EMA(Vector3 new_val, Vector3 old_val, float alpha) {
	if (alpha > 1) alpha = 1.0f;
	Vector3 ema_val;
	ema_val.x = EMA(new_val.x, old_val.x, alpha);
	ema_val.y = EMA(new_val.y, old_val.y, alpha);
	ema_val.z = EMA(new_val.z, old_val.z, alpha);
	return ema_val;
}

// #TODO: Make matrix for transforms
// void Body2World(Vector3 euler) {
// 	float& r      = euler.x;
// 	float& p      = euler.y;
// 	float& y      = euler.z;
// 	float R[3][3] = {
// 			{															cosf(p) * cosf(y),															 cosf(p) * cosf(y),          -sinf(p)},
// 			{										sinf(r) * sinf(p) * cosf(y), sinf(r) * sinf(p) * sinf(y) + cosf(r) * cosf(p), sinf(r) * cosf(p)},
// 			{cosf(r) * sinf(p) * cosf(y) + sinf(r) * sinf(y), cosf(r) * sinf(p) * sinf(y) - sinf(r) * cosf(y), cosf(r) * cosf(p)},
// 	};
// }

Vector2 EulerFromAccel(Vector3 accelerometer) {
	float x2      = accelerometer.x * accelerometer.x;
	float y2      = accelerometer.y * accelerometer.y;
	float z2      = accelerometer.z * accelerometer.z;
	float gVector = sqrtf(x2 + y2 + z2);
	float roll    = atan2f(accelerometer.y, accelerometer.z);
	float pitch   = asinf(accelerometer.x / gVector);
	if (pitch == INFINITY) pitch = 0.0f;
	return Vector2{roll, pitch};
}
