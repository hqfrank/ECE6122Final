//
// Created by qiang on 5/2/18.
//

#ifndef FINALPROJECT_UTILITYMETHODS_H
#define FINALPROJECT_UTILITYMETHODS_H

#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <random>
#include <memory>
#include <chrono>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "Building_t.h"
#include "SystemParameters.h"
#include "Plane_t.h"

double GetMin(const std::vector<double>& vec);

template <typename T>
std::vector<int> sort_indexes(const std::vector<T> &v) {

    // initialize original index locations
    std::vector<int> idx(v.size());
    iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    sort(idx.begin(), idx.end(),
         [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

    return idx;
}

#endif //FINALPROJECT_UTILITYMETHODS_H

