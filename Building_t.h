//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_BUILDING_T_H
#define FINALPROJECT_BUILDING_T_H

#include "Point_t.h"
#include <vector>
#include <cmath>
#include <random>

class Point_t;

class Building_t {
private:
  /* === Member variables === */
  Point_t* Center;    // Center of the building from the top view, i.e., (x,y,0)
  double Length;			// length of the building from the top view
  double Width;			  // width of the building from the top view
  double Height;			// top level of the building
  double HeightBase;	// the lowest height above which the candidate relays will be deployed
  double GroundLevel;	// the height of the ground level of the building
  double Orientation;	// orientation (the angle between the "Length" side and north) in radius

  std::vector<Point_t*> Vbs;	     // vertices at the base height

  std::vector<Point_t*> Vgs;       // vertices at the ground level

  std::vector<Point_t*> Vts;       // vertices at the top of the building

  std::vector<Point_t*> Relays;   // vector to store all relays on a building.

  int NumRelays;

  bool hasBS;

public:
  /* === Member methods === */
  /* ------ Constructors ------ */
  Building_t(double center[], double lwhbg[], double orientation, double threshold, double density, int randomSeed);
  /* ------ Methods ------ */
  /* Function to generate 12 vertices on the building. 4 of them are on the plane with the minimum height to deploy relays. */
  void VertexGenerator();
  /* Generate randomly (uniformly) deployed relays on the building surface. */
  std::vector<Point_t*> GenerateRelayOnFace(Point_t& va, Point_t& vb, Point_t& vc, Point_t& vd, double density, int randomSeed);
  /* Generate relays on a whole building. */
  std::vector<Point_t*> RelayGenerator(double threshold, double density, int randomSeed);
  /* Get member variables. */
  std::vector<Point_t*> getVbs();
  std::vector<Point_t*> getVgs();
  std::vector<Point_t*> getVts();
  std::vector<Point_t*> getRelays();
  double getHeight();
  double getHeightBase();
  /* Set member variables. */
  void setHasBS(bool has);



  /* Print out string */
  std::string toString();
  std::string toStringData();





};


#endif //FINALPROJECT_BUILDING_T_H