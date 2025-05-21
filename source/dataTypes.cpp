#include "dataTypes.hpp"

#include <cmath>

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
