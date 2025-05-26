#include "math.hpp"

// Scalar Arithmetic

Vector3 Vector3::operator-() const {
	return Vector3{-x, -y, -z};
}

Vector3 Vector3::operator+(float rhs) const {
	return Vector3{x + rhs, y + rhs, z + rhs};
}
Vector3 Vector3::operator-(float rhs) const {
	return Vector3{x - rhs, y - rhs, z - rhs};
}
Vector3 Vector3::operator*(float rhs) const {
	return Vector3{x * rhs, y * rhs, z * rhs};
}
Vector3 Vector3::operator/(float rhs) const {
	return Vector3{x / rhs, y / rhs, z / rhs};
}
Vector3 Vector3::operator%(float rhs) const {
	return Vector3{(float)fmod(x, rhs), (float)fmod(x, rhs), (float)fmod(x, rhs)};
}

Vector3& Vector3::operator-=(float rhs) {
	x -= rhs;
	y -= rhs;
	z -= rhs;
	return *this;
}
Vector3& Vector3::operator*=(float rhs) {
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
}
Vector3& Vector3::operator/=(float rhs) {
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}
Vector3& Vector3::operator%=(float rhs) {
	x = (float)fmod(x, rhs);
	y = (float)fmod(y, rhs);
	z = (float)fmod(z, rhs);
	return *this;
}

// Vector Arithmetic

Vector3 Vector3::operator+(Vector3& rhs) const {
	return Vector3{x + rhs.x, y + rhs.y, z + rhs.z};
}
Vector3 Vector3::operator-(Vector3& rhs) const {
	return Vector3{x - rhs.x, y - rhs.y, z - rhs.z};
}

Vector3 Vector3::operator+(Vector3 rhs) const {
	return Vector3{x + rhs.x, y + rhs.y, z + rhs.z};
}

Vector3& Vector3::operator+=(Vector3 rhs) {
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}
Vector3& Vector3::operator-=(Vector3 rhs) {
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

// Methods

void Vector3::Round(int digits) {
	for (int i = 0; i < digits; i++) *this *= 10;
	x = round(x);
	y = round(y);
	z = round(z);
	for (int i = 0; i < digits; i++) *this /= 10;
}

float Vector3::Magnitude() const {
	float m = sqrtf(x * x + y * y + z * z);
	return m == INFINITY ? 0 : m;
}

float Vector3::Dot(const Vector3& rhs) const {
	return x * rhs.x + y * rhs.y + z * rhs.z;
}
Vector3 Vector3::Cross(const Vector3& rhs) const {
	return Vector3{
			y * rhs.z - z * rhs.y,
			z * rhs.x - x * rhs.z,
			x * rhs.y - y * rhs.x};
}

Vector3& Vector3::Normalize() {
	*this /= Magnitude();
	return *this;
}

Quaternion Vector3::ToQuaternion() {
	float r2 = x / 2, p2 = y / 2, y2 = z / 2;
	float cr = cosf(r2), sr = sinf(r2);
	float cp = cosf(p2), sp = sinf(p2);
	float cy = cosf(y2), sy = sinf(y2);

	Quaternion q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;
	return q;
}

Vector3 Normalize(const Vector3& vec) {
	float m = vec.Magnitude();
	return Vector3{vec.x / m, vec.y / m, vec.z / m};
}
