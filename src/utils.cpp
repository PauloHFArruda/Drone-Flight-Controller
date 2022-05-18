#include "utils.h"

double sat(double val, double min, double max) {
    return val < min ? min : (val > max ? max : val);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}