//
// Created by qiang on 4/16/18.
//

#include "Plane_t.h"

Plane_t::Plane_t(const Vector_t &n, const Point_t &p) {
  this->normal = Vector_t(n.divide(n.mod()));	// normalized normal of the plane
  this->dVal = -1 * this->normal.dot(p);
  this->v1 = p;
  this->v2 = Point_t();
  this->v3 = Point_t();
  this->v4 = Point_t();
}

Plane_t::Plane_t(const Point_t& p1, const Point_t& p2, const Point_t& p3, const Point_t& p4){
  Vector_t v12 (p1, p2);
  Vector_t v13 (p1, p3);
  this->normal = normalize(cross(v12,v13));
  this->dVal = -1 * (this->normal.dot(p1));
  this->v1 = p1;
  this->v2 = p2;
  this->v3 = p3;
  this->v4 = p4;
}

Point_t Plane_t::planeIntersectLine(const Line_t& l) const {
  /*
   * Determine whether the angle between the normal of the plane
   * and the normalized direction vector of the line l is orthogonal.
   * Due to the calculation precision, we consider it is orthogonal
   * when 89.99^o < angle < 90.99^o.
   */
  if(std::abs(dot(this->normal, l.getDir())) < TEST_ORTHOGONAL){
    Point_t pNull;
    return pNull; // no intersection
    /*
     * Line segment l is parallel to the plane (out of plane) or in the plane.
     * Since in our application, we assume the buildings as cubics, thus,
     * two nodes on the same face can communicate with each other directly.
     * In another word, the line is in the plane but not "intersected".
     * (There are infinite number of intersect points.)
     */
  }
  else{
    /*
     * Refer to http://geomalgorithms.com/a05-_intersect-1.html
     */
    Vector_t w (this->v1, l.getSrc());
    double den = dot(this->normal, l.getSd());
    double num = -1 * dot(this->normal, w);
    double sI = num/den;
    // System.out.println(sI);
    if (sI < (0 - TEST_ERROR) || sI > (1 + TEST_ERROR)){
      /*
       * If the intersect point is not close to src or dst,
       * the line segment l does not intersect with the plane.
       */
      Point_t pNull;
      return pNull; // no intersection
    }
    else if(std::abs(sI-1) <= TEST_ERROR || std::abs(sI) <= TEST_ERROR){
      /*
       * If the intersect point is very close to src or dst
       * Due to our usage, the line segment starts from a node
       * and ends at another node. If the intersect point is
       * either src or dst, the line segment and the plane is
       * not recognized as intersected. Thus, we use a special
       * intersect point to denote this case (i.e., one of the
       * end points of the line segment is in the plane).
       */
      if (pointInRectangle(l.getSrc())){
        return l.getSrc(); // the intersect point is src, and dst is not on the face
      }
      if (pointInRectangle(l.getDst())){
        return l.getDst(); // the intersect point is dst, and src is not on the face
      }
      Point_t pNull;
      return pNull;
    }
    else{
      /*
       * If the intersect point is neither src nor dst,
       * calculate the intersect point.
       */
      Point_t iPoint = (l.getSrc()).destinationTo(Vector_t(l.getSd(), sI));
      if (pointInRectangle(iPoint)){
        return iPoint;
      }
      else{
        Point_t pNull;
        return pNull;
      }
    }
  }
}

bool Plane_t::pointInRectangle(const Point_t& p) const {
  /*
   * Project these 5 points (v1, v2, v3, v4, p) into the rectangle.
   * Set v1 as the origin (0,0), vector v1->v2 is x axis, v1->v4 is y axis
   */
  Vector_t op(this->v1, p);
  Vector_t ox(this->v1, this->v2);
  Vector_t oy(this->v1, this->v4);
  double px = (ox.dot(op))/ox.mod();	// the x value of p
  double py = (oy.dot(op))/oy.mod();	// the y value of p

  if(px >= (-1 * TEST_ERROR) && px <= (ox.mod()+TEST_ERROR) && py >= (-1 * TEST_ERROR) && py <= (oy.mod()+TEST_ERROR)){
    return true;
  }
  else{
    return false;
  }
}

Vector_t Plane_t::getNormal() const { return this->normal; }

double Plane_t::getDval() const { return this->dVal; }

Point_t Plane_t::getV1() const { return this->v1; }

Point_t Plane_t::getV2() const { return this->v2; }

Point_t Plane_t::getV3() const { return this->v3; }

Point_t Plane_t::getV4() const { return this->v4; }

std::string Plane_t::toString() const {
  return "Plane has a normal as " + this->normal.toString() + " and goes through point " + this->v1.toString();
}