//
// Created by qiang on 4/16/18.
//

#include "Vector_t.h"

/* === Member methods === */
/* ------ Constructors ------ */
/* Use the coordinate of a vector to construct the new vector. */
Vector_t::Vector_t(double x, double y, double z){
  this->x = x;
  this->y = y;
  this->z = z;
}
/* Use the instance of another vector to construct the new vector. */
Vector_t::Vector_t(Vector_t& v){
  this->x = v.getX();
  this->y = v.getY();
  this->z = v.getZ();
}
/* Use the locations of two points s and d to construct the new vector. (from s to d) */
Vector_t::Vector_t(Point_t& s, Point_t& d){
  this->x = d.getX() - s.getX();
  this->y = d.getY() - s.getY();
  this->z = d.getZ() - s.getZ();
}
/* Construct a new vector which is 'a' times the vector 'v'. */
Vector_t::Vector_t(Vector_t& v, double a){
  this->x = a * v.getX();
  this->y = a * v.getY();
  this->z = a * v.getZ();
}
/* Construct a new vector based on the angle to XY plane and the angle to the y+ axis in XY plane. */
Vector_t::Vector_t(double angleToXY_degree, double angleInXY_degree){
  double lengthInXY = 1.0 * cos(angleToXY_degree/180.0*M_PI);
  this->x = lengthInXY * sin(angleInXY_degree/180.0*M_PI);
  this->y = lengthInXY * cos(angleInXY_degree/180.0*M_PI);
  this->z = 1.0 * sin(angleToXY_degree/180.0*M_PI);
}

/* ------ Methods ------ */
/* Get the x,y,z values */
double Vector_t::getX(){
  return this->x;
}
double Vector_t::getY(){
  return this->y;
}
double Vector_t::getZ(){
  return this->z;
}
/* Set each coordinate. */
void Vector_t::setX(double newX){
  this->x = newX;
}
void Vector_t::setY(double newY){
  this->y = newY;
}
void Vector_t::setZ(double newZ){
  this->z = newZ;
}
/* Length of the vector */
double Vector_t::mod(){
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}

double Vector_t::length(){
  return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}
Vector_t* Vector_t::plus(Vector_t& v){
  return new Vector_t(this->x + v.x, this->y + v.y, this->z + v.z);
}
Vector_t* Vector_t::minus(Vector_t& v){
  return new Vector_t(this->x - v.x, this->y - v.y, this->z - v.z);
}
Vector_t* Vector_t::divide(double d){
  assert(d != 0);
  return new Vector_t(this->x / d, this->y / d, this->z / d);
}
Vector_t* Vector_t::times(double d){
  return new Vector_t(this->x * d, this->y * d, this->z * d);
}
double Vector_t::dot(Vector_t& v){
  return (this->x * v.getX()) + (this->y * v.getY()) + (this->z * v.getZ());
}
double Vector_t::dot(Point_t& p){
  return (this->x * p.getX()) + (this->y * p.getY()) + (this->z * p.getZ());
}
/* Cross product: u x v = (u2v3-u3v2, u3v1-u1v3, u1v2-u2v1) */
Vector_t* Vector_t::cross(Vector_t& v){
  return new Vector_t(this->y * v.getZ() - this->z * v.getY(), this->z * v.getX() - this->x * v.getZ(), this->x * v.getY() - this->y * v.getX());
}
string Vector_t::toString(){
  return "Vector [" + to_string(this->x) + "," + to_string(this->y) + "," + to_string(this->z) + "] Length: " + to_string(this->mod());
}

