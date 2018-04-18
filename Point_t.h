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

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  /* Use the coordinate of a point to construct the new point. */
  Point_t(double x, double y, double z);
  /* Use the instance of another point to construct the new point. */
  Point_t(Point_t& p);
  /* ------ Methods ------ */
  /* Get the x,y,z values */
  double getX();
  double getY();
  double getZ();
  /* Set each coordinate. */
  void setX(double newX);
  void setY(double newY);
  void setZ(double newZ);

  /*
   * Calculate the distance from 'this' point to point 'p'.
   * (this) *---?---* (p)
   */
  double distanceTo(Point_t& p);
  /*
   * Calculate the location of a new point which is the destination given a vector.
   * (this) *---dir--->?
   */
  Point_t* destinationTo(Vector_t& dir);
  /*
   * Calculate the location of a source point from which a vector dir is given to this point.
   * ? ---dir--->* (this)
   */
  Point_t* sourceFrom(Vector_t& dir);
  /* Calculate the vector from this point to point 'p'. */
  Vector_t* vectorTo(Point_t& p);
  /* Determine whether (this == p) is true. */
  bool sameAs(Point_t& p);
  /* Print out string */
  string toString();
  string toStringData();
};


#endif //FINALPROJECT_POINT_T_H