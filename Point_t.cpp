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
}

/* Use the instance of another point to construct the new point. */
Point_t::Point_t(Point_t& p){
  this->x = p.getX();
  this->y = p.getY();
  this->z = p.getZ();
}
/* ------ Methods ------ */
/* Get the x,y,z values */
double Point_t::getX(){
  return this->x;
}
double Point_t::getY(){
  return this->y;
}
double Point_t::getZ(){
  return this->z;
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

/*
 * Calculate the distance from 'this' point to point 'p'.
 * (this) *---?---* (p)
 */
double Point_t::distanceTo(Point_t& p){
  return sqrt( pow(this->x - p.getX(), 2.0) + pow(this->y - p.getY(), 2.0) + pow(this->z - p.getZ(), 2.0) );
}
/*
 * Calculate the location of a new point which is the destination given a vector.
 * (this) *---dir--->?
 */
Point_t* Point_t::destinationTo(Vector_t& dir){
  return new Point_t(this->x + dir.getX(), this->y + dir.getY(), this->z + dir.getZ());
}
/*
 * Calculate the location of a source point from which a vector dir is given to this point.
 * ? ---dir--->* (this)
 */
Point_t* Point_t::sourceFrom(Vector_t& dir){
  return new Point_t(this->x - dir.getX(), this->y - dir.getY(), this->z - dir.getZ());
}
/* Calculate the vector from this point to point 'p'. */
Vector_t* Point_t::vectorTo(Point_t& p){
  return new Vector_t(*this, p);
}
/* Determine whether (this == p) is true. */
bool Point_t::sameAs(Point_t& p){
  if(abs(this->getX() - p.getX()) > 1E-5) return false;
  if(abs(this->getY() - p.getY()) > 1E-5) return false;
  if(abs(this->getZ() - p.getZ()) > 1E-5) return false;
  return true;
}
/* Print out string */
string Point_t::toString(){
  return "Point [" + to_string(this->x) + "," + to_string(this->y) + "," + to_string(this->z) + "]";
}