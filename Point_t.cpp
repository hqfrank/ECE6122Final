//
// Created by qiang on 4/16/18.
//

#include "Point_t.h"

/* === Member methods === */
/* ------ Constructors ------ */
/* Use the coordinate of a point to construct the new point. */
Point_t::Point_t(double x, double y, double z){
  this->x = x;
  this->y = y;
  this->z = z;
  this->valid = true;
}

/* Use the instance of another point to construct the new point. */
Point_t::Point_t(const Point_t& p){
  this->x = p.getX();
  this->y = p.getY();
  this->z = p.getZ();
  this->valid = p.getValid();
}
/* A default constructor to construct a null point object. */
Point_t::Point_t() {
  this->valid = false;
}
/* ------ Methods ------ */
/* Get the x,y,z values */
double Point_t::getX() const {
  return this->x;
}
double Point_t::getY() const {
  return this->y;
}
double Point_t::getZ() const {
  return this->z;
}
bool Point_t::getValid() const {
  return this->valid;
}
/* Set each coordinate. */
void Point_t::setX(double newX){
  this->x = newX;
}
void Point_t::setY(double newY){
  this->y = newY;
}
void Point_t::setZ(double newZ){
  this->z = newZ;
}
void Point_t::setValid(bool val) {
  this->valid = val;
}

/*
 * Calculate the distance from 'this' point to point 'p'.
 * (this) *---?---* (p)
 */
double Point_t::distanceTo(const Point_t& p) const {
  return sqrt( pow(this->x - p.getX(), 2.0) + pow(this->y - p.getY(), 2.0) + pow(this->z - p.getZ(), 2.0) );
}
/*
 * Calculate the location of a new point which is the destination given a vector.
 * (this) *---dir--->?
 */
Point_t Point_t::destinationTo(const Vector_t& dir) const {
  Point_t p(this->x + dir.getX(), this->y + dir.getY(), this->z + dir.getZ());
  return p;
}
/*
 * Calculate the location of a source point from which a vector dir is given to this point.
 * ? ---dir--->* (this)
 */
Point_t Point_t::sourceFrom(const Vector_t& dir) const{
  Point_t p(this->x - dir.getX(), this->y - dir.getY(), this->z - dir.getZ());
  return p;
}
/* Calculate the vector from this point to point 'p'. */
Vector_t Point_t::vectorTo(const Point_t& p) const {
  Vector_t v(*this, p);
  return v;
}
/* Determine whether (this == p) is true. */
bool Point_t::sameAs(const Point_t& p) const {
  if(abs(this->getX() - p.getX()) > 1E-5) return false;
  if(abs(this->getY() - p.getY()) > 1E-5) return false;
  if(abs(this->getZ() - p.getZ()) > 1E-5) return false;
  return true;
}
/* Print out string */
string Point_t::toString() const {
  return "Point [" + to_string(this->x) + "," + to_string(this->y) + "," + to_string(this->z) + "]";
}

string Point_t::toStringData() const {
  return to_string(this->x) + "," + to_string(this->y) + "," + to_string(this->z);
}