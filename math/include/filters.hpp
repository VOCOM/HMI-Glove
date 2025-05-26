#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "types.hpp"

/**
 * @brief Complementary Filter
 *
 * @param new_val
 * @param old_val
 * @param alpha
 * @return float
 */
float CF(float lhs, float rhs, float alpha);

/**
 * @brief Complementary Filter
 *
 * @param new_val
 * @param old_val
 * @param alpha
 * @return Vector3
 */
Vector3 CF(Vector3 lhs, Vector3 rhs, float alpha);

/**
 * @brief Kalman Filter
 *
 */
class KF {
public:
	void Predict(Quaternion& attitude, Vector3 gyro, Vector3 accel, float dt);

private:
	Vector3 gyroBias{};
	float P[7][7] = {0};

	const float Q_proc = 0.001f;
	const float Q_bias = 0.003f;

	const float R_accel = 0.1f;
};

#endif /* FILTERS */
