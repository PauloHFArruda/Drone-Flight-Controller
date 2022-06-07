#ifndef UTILS_H
	#define UTILS_H

	double sat(double val, double min, double max);
	float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
	float strToFloat(char str[]);
	void copyFloatArray(float src[], float dst[], int len);

#endif // !UTILS_H