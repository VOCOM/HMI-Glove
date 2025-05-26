#include "math.hpp"

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
	float &r = angles.x, &p = angles.y, &y = angles.z;
	float cr = cosf(r), cp = cosf(p), cy = cosf(y);
	float sr = sinf(r), sp = sinf(p), sy = sinf(y);

	Matrix3x3 newMat{};
	newMat[0] = cp * cy;
	newMat[1] = sr * sp * cy - cr * sy;
	newMat[2] = cr * sp * cy + sr * sy;
	newMat[3] = cp * sy;
	newMat[4] = sr * sp * sy + cr * cy;
	newMat[5] = cr * sp * sy - sr * cy;
	newMat[6] = -sp;
	newMat[7] = sr * cp;
	newMat[8] = cr * cp;
	return newMat;
}

float& Matrix4x4::operator[](int index) {
	return data[index];
}
