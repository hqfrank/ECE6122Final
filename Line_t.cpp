//
// Created by qiang on 4/16/18.
//

#include "Line_t.h"

Line_t::Line_t(const Point_t& s, const Point_t& d) {
  this->src = Point_t(s);
  this->dst = Point_t(d);
  this->sd = Vector_t(s,d);
  this->dir = normalize();
}

Point_t Line_t::getSrc() const {
  return this->src;
}

Point_t Line_t::getDst() const {
  return this->dst;
}

Vector_t Line_t::getSd() const {
  return this->sd;
}

Vector_t Line_t::getDir() const {
  return this->dir;
}

Vector_t Line_t::normalize(){
  Vector_t v = this->sd.divide(this->sd.mod());
  return v;
}

std::string Line_t::toString() const {
  return "Line " + this->src.toString() + " to " + this->dst.toString() + " has normalized direction " + this->dir.toString();
}