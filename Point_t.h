//
// Created by qiang on 4/16/18.
//


#ifndef FINALPROJECT_POINT_T_H
#define FINALPROJECT_POINT_T_H

#include <cmath>
#include <string>
#include "Vector_t.h"

using namespace std;

class Vector_t;

class Point_t {

private:
  /* === Member variables === */
  /* Define the coordinate of a Point as [x,y,z]. */
  double x;
  double y;
  double z;
  bool valid;

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  /* Use the coordinate of a point to construct the new point. */
  Point_t(double x, double y, double z);
  /* Use the instance of another point to construct the new point. */
  Point_t(const Point_t& p);
  /* Default constructor */
  Point_t();
  /* ------ Methods ------ */
  /* Get the x,y,z values */
  double getX() const;
  double getY() const;
  double getZ() const;
  bool getValid() const;
  /* Set each coordinate. */
  void setX(double newX);
  void setY(double newY);
  void setZ(double newZ);
  void setValid(bool val);

  /*
   * Calculate the distance from 'this' point to point 'p'.
   * (this) *---?---* (p)
   */
  double distanceTo(const Point_t& p) const;

  /*
   * Calculate the location of a new point which is the destination given a vector.
   * (this) *---dir--->?
   */
  Point_t destinationTo(const Vector_t& dir) const;

  /*
   * Calculate the location of a source point from which a vector dir is given to this point.
   * ? ---dir--->* (this)
   */
  Point_t sourceFrom(const Vector_t& dir) const;

  /* Calculate the vector from this point to point 'p'. */
  Vector_t vectorTo(const Point_t& p) const;

  /* Determine whether (this == p) is true. */
  bool sameAs(const Point_t& p) const;

  /* Print out string */
  std::string toString() const;
  std::string toStringData() const;
};


#endif //FINALPROJECT_POINT_T_H