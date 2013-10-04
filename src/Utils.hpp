#ifndef UTILS_HPP
#define UTILS_HPP

/* partDone will be normalized in [0,1] */
double crossfadedValue(double src, double dst, double partDone);

double boundDouble(double val, double min, double max);

bool isZero(double val);

#endif//UTILS_HPP
