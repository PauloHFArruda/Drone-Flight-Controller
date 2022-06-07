#include "utils.h"

float sat(float val, float min, float max) {
    return val < min ? min : (val > max ? max : val);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

float strToPositiveFloat(char str[]) {
    float val = 0, aux = 1;
    int i = 0;

    while (str[i] != '\0' && str[i] != '.') { i++; }

    for (int j = i-1; j >= 0; j--) {
      val += (str[j] - '0')*aux;
      aux *= 10;
    }

    aux = 0.1;
    for (int j = i+1; str[j] != '\0'; j++) {
      val += (str[j] - '0')*aux;
      aux *= 0.1;
    }

    return val;
}

float strToFloat(char str[]) {
    return str[0] == '-' ? -strToPositiveFloat(str+1) : strToPositiveFloat(str);
}

void copyFloatArray(float src[], float dst[], int len) {
    for (int i = 0; i < len; i++)
        dst[i] = src[i];
}