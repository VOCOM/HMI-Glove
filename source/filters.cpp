#include "filters.hpp"

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
