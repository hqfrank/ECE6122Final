//
// Created by qiang on 4/16/18.
//

#include "Line_t.h"

Line_t::Line_t(Point_t& s, Point_t& d){
  this->src = new Point_t(s);
  this->dst = new Point_t(d);
  this->sd = new Vector_t(s,d);
  this->dir = normalize();
}

Point_t* Line_t::getSrc() {
  return this->src;
}

Point_t* Line_t::getDst() {
  return this->dst;
}

Vector_t* Line_t::getSd() {
  return this->sd;
}

Vector_t* Line_t::getDir() {
  return this->dir;
}

Vector_t* Line_t::normalize(){
  return this->sd->divide(this->sd->mod());
}

std::string Line_t::toString() {
  return "Line " + this->src->toString() + " to " + this->dst->toString() + " has normalized direction " + this->dir->toString();
}