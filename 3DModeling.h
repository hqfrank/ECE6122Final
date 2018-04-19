//
// Created by qiang on 4/16/18.
//

#ifndef FINALPROJECT_3DMODELING_H
#define FINALPROJECT_3DMODELING_H

#include <iostream>
#include <fstream>
#include <cassert>
#include <cmath>
#include <random>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "Building_t.h"
#include "SystemParameters.h"
#include "Plane_t.h"


Point_t* proportionalPoint(Point_t& p1, Point_t& p2, double pro);

double dot(Vector_t& v1, Vector_t& v2);

Vector_t* cross(Vector_t& v1, Vector_t& v2);

Vector_t* normalize(Vector_t& v);

std::vector<Building_t*> getBuildingInfoFromFile(std::string dataBuildings, std::string dataBuildingVertices, SystemParameters parameters);

/* Generate candidate base station locations randomly, and it also generates the locations of all relays located at the roof top corners. */
std::vector<Point_t*> generateCandidateBaseStations(std::vector<Building_t*> buildingSet, std::vector<Point_t*>& roofTopRelays, SystemParameters parameters);

/* Select base stations based on the grid constraint. */
void selectBaseStationPerGrid(std::vector<Point_t*>& bsSet, SystemParameters parameters);

/* Collect all relays on the surfaces of buildings. */
std::vector<Point_t*> collectAllRelays(std::vector<Building_t*> buildings);

/* Select roof top relays based on the grid constraint. */
void selectRelayPerGrid(std::vector<Point_t*>& relays, SystemParameters parameters);

/* Explore the connectivity between each pair of nodes in the topology. */
std::vector<std::vector<int>> exploreConnectivity(std::vector<Point_t*>& nodes, std::vector<Building_t*>& buildings, const std::string& fileRelayNeighbors);

/* Evaluate the line-of-sight connectivity between a point s and other nodes in the topology. */
std::vector<int> searchNonBlockLink(std::vector<Building_t*>& buildings, Point_t& s, std::vector<Point_t*>& nodes);

/* Test whether the line sd is intersect with any building in the building set. */
bool blockageTest(const std::vector<Building_t*>& buildingSet, Line_t& sd);

#endif //FINALPROJECT_3DMODELING_H
