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
  Vector_t(const Vector_t& v);
  /* Use the locations of two points s and d to construct the new vector. (from s to d) */
  Vector_t(const Point_t& s, const Point_t& d);
  /* Construct a new vector which is 'a' times the vector 'v'. */
  Vector_t(const Vector_t& v, double a);
  /* Construct a new vector based on the angle to XY plane and the angle to the y+ axis in XY plane. */
  Vector_t(double angleToXY_degree, double angleInXY_degree);
  /* Default constructor */
  Vector_t();

  /* ------ Methods ------ */
  /* Get the x,y,z values */
  double getX() const;
  double getY() const;
  double getZ() const;
  /* Set each coordinate. */
  void setX(double newX);
  void setY(double newY);
  void setZ(double newZ);
  /* Length of the vector */
  double mod() const;
  double length() const;
  Vector_t plus(const Vector_t& v) const;
  Vector_t minus(const Vector_t& v) const;
  Vector_t divide(double d) const;
  Vector_t times(double d) const;
  double dot(const Vector_t& v) const;
  double dot(const Point_t& p) const;
  /* Cross product: u x v = (u2v3-u3v2, u3v1-u1v3, u1v2-u2v1) */
  Vector_t cross(const Vector_t& v) const;
  /* Print out strings */
  std::string toString() const;
  std::string toStringData() const;


};


#endif //FINALPROJECT_VECTOR_T_H