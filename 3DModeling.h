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
#include <memory>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "Building_t.h"
#include "SystemParameters.h"
#include "Plane_t.h"


Point_t proportionalPoint(const Point_t& p1, const Point_t& p2, double pro);

double dot(const Vector_t& v1, const Vector_t& v2);

Vector_t cross(const Vector_t& v1, const Vector_t& v2);

Vector_t normalize(const Vector_t& v);

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

/* Read the connectivity information from file. */
void getRelayNeighborInfoFromFile(std::vector<std::vector<int>>& relayNeighborList, const std::string dataRelayNeighbors);

#endif //FINALPROJECT_3DMODELING_H
