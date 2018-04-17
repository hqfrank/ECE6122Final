//
// Created by qiang on 4/16/18.
//

#include "3DModeling.h"

/*
 * Calculate the point at the position p between p1 and p2, such that |p1p|= pro*|p1p2|
 */
Point_t* proportionalPoint(Point_t& p1, Point_t& p2, double pro){
  assert(pro >= 0);
  Vector_t* v12 = new Vector_t(p1, p2);
  Vector_t* v = new Vector_t(*v12, pro);
  return p1.destinationTo(*v);
}

/*
 * Dot product of two vectors
 */
double dot(Vector_t& v1, Vector_t& v2){
  return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY()) + (v1.getZ() * v2.getZ());
}

Vector_t* cross(Vector_t& v1, Vector_t& v2){
  return new Vector_t(v1.getY() * v2.getZ() - v1.getZ() * v2.getY(), v1.getZ() * v2.getX() - v1.getX() * v2.getZ(), v1.getX() * v2.getY() - v1.getY() * v2.getX());
}

Vector_t* normalize(Vector_t& v){
  return v.divide(v.mod());
}
