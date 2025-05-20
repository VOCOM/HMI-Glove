#ifndef FILTERS_HPP
#define FILTERS_HPP

#include "dataTypes.hpp"

float EMA(float new_val, float old_val, float alpha);
Vector3 EMA(Vector3 new_val, Vector3 old_val, float alpha);

#endif /* FILTERS */
