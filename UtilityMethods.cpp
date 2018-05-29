//
// Created by qiang on 5/2/18.
//

#include "UtilityMethods.h"

double GetMin (const std::vector<double>& vec) {
  double min = vec.at(0);
  for (auto num : vec) {
    if (num < min)
      min = num;
  }
  return min;
}

