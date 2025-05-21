#include "dataTypes.hpp"

Vector3 Vector3::operator-(float rhs) {
	return Vector3{this->x - rhs, this->y - rhs, this->z - rhs};
}

Vector3 Vector3::operator*(float rhs) {
	return Vector3{this->x * rhs, this->y * rhs, this->z * rhs};
}

Vector3 Vector3::operator/(float rhs) {
	return Vector3{this->x / rhs, this->y / rhs, this->z / rhs};
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
