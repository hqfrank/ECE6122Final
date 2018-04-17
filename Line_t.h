//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_LINE_T_H
#define FINALPROJECT_LINE_T_H

#include "Point_t.h"
#include "Vector_t.h"
#include "3DModeling.h"

class Point_t;
class Vector_t;

class Line_t {
private:
  /* === Member variables === */
  Point_t* src;	// source point
  Point_t* dst;	// destination point
  Vector_t* sd;	// the vector from src to dst
  Vector_t* dir;	// normalized directional vector from src to dst

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  Line_t(Point_t& s, Point_t& d);
  /* ------ Methods ------ */
  /* Get member variables */
  Point_t* getSrc();
  Point_t* getDst();
  Vector_t* getSd();
  Vector_t* getDir();
  /* Print out string */
  std::string toString();
};


#endif //FINALPROJECT_LINE_T_H