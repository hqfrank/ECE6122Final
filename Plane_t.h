//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_PLANE_T_H
#define FINALPROJECT_PLANE_T_H

#include "Vector_t.h"
#include "Point_t.h"
#include "Line_t.h"
#include "3DModeling.h"
#include <cmath>

class Vector_t;
class Point_t;
class Line_t;

#ifndef TEST_ORTHOGONAL
#define TEST_ORTHOGONAL std::abs(cos((90.0 - 0.01) / 180.0 * M_PI))
#endif

#ifndef TEST_ERROR
#define TEST_ERROR 1E-5
#endif


class Plane_t {
private:
  /* === Member variables === */
  Vector_t normal;		// The normal of this plane
  double dVal;			// The value of d in the plane equation ax+by+cz+d = 0
  Point_t v1;			// A vertex on the plane
  Point_t v2;
  Point_t v3;
  Point_t v4;

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  /* Given the normal n of the plane and a point p in the plane. */
  Plane_t(const Vector_t& n, const Point_t& p);
  /* Given 4 nodes of a plane. */
  Plane_t(const Point_t& p1, const Point_t& p2, const Point_t& p3, const Point_t& p4);

  /* ------ Methods ------ */
  /*
   * Find the intersect point between a line segment and a plane.
   */
  Point_t planeIntersectLine(const Line_t& l) const;
  /*
   * Determine whether a point is in a rectangle area.
   * It is known that the point is in the same plane of
   * the rectangle defined by the four vertices:
   * v1, v2, v3, and v4.
   */
  bool pointInRectangle(const Point_t& p) const;
  /* Get members */
  Vector_t getNormal() const;
  double getDval() const;
  Point_t getV1() const;
  Point_t getV2() const;
  Point_t getV3() const;
  Point_t getV4() const;
  /* Print out string */
  std::string toString() const;

};


#endif //FINALPROJECT_PLANE_T_H