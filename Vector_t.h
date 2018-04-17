//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_VECTOR_T_H
#define FINALPROJECT_VECTOR_T_H

#include <cmath>
#include <cassert>
#include <string>
#include "Point_t.h"

class Point_t;

class Vector_t {
private:
  /* === Member variables === */
  /* Define the coordinate of a vector as [x,y,z]. */
  double x;
  double y;
  double z;

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  /* Use the coordinate of a vector to construct the new vector. */
  Vector_t(double x, double y, double z);
  /* Use the instance of another vector to construct the new vector. */
  Vector_t(Vector_t& v);
  /* Use the locations of two points s and d to construct the new vector. (from s to d) */
  Vector_t(Point_t& s, Point_t& d);
  /* Construct a new vector which is 'a' times the vector 'v'. */
  Vector_t(Vector_t& v, double a);
  /* Construct a new vector based on the angle to XY plane and the angle to the y+ axis in XY plane. */
  Vector_t(double angleToXY_degree, double angleInXY_degree);

  /* ------ Methods ------ */
  /* Get the x,y,z values */
  double getX();
  double getY();
  double getZ();
  /* Set each coordinate. */
  void setX(double newX);
  void setY(double newY);
  void setZ(double newZ);
  /* Length of the vector */
  double mod();
  double length();
  Vector_t* plus(Vector_t& v);
  Vector_t* minus(Vector_t& v);
  Vector_t* divide(double d);
  Vector_t* times(double d);
  double dot(Vector_t& v);
  double dot(Point_t& p);
  /* Cross product: u x v = (u2v3-u3v2, u3v1-u1v3, u1v2-u2v1) */
  Vector_t* cross(Vector_t& v);
  std::string toString();


};


#endif //FINALPROJECT_VECTOR_T_H