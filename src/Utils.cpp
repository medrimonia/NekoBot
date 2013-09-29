#include "Utils.hpp"

#define EPSILON 0.0001

double boundDouble(double val, double min, double max){
  if (val < min){
    return min;
  }
  if (val > max){
    return max;
  }
  return val;
}

bool isZero(double val){
  if (val > 0){
    return val < EPSILON;
  }
  return val > -EPSILON;
}
