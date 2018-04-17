//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_3DMODELING_H
#define FINALPROJECT_3DMODELING_H

#include <iostream>
#include <cassert>
#include <cmath>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"


Point_t* proportionalPoint(Point_t& p1, Point_t& p2, double pro);

double dot(Vector_t& v1, Vector_t& v2);

Vector_t* cross(Vector_t& v1, Vector_t& v2);

Vector_t* normalize(Vector_t& v);

#endif //FINALPROJECT_3DMODELING_H
