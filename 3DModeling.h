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
#include <chrono>
#include <algorithm>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "Building_t.h"
#include "SystemParameters.h"
#include "Plane_t.h"
#include "Path_t.h"
#include "UtilityMethods.h"

class EstimatedHop {
public:
  const double l200g20 = 1*0.48+2*0.51+3*0.01;
  const double l400g200 = 1*0.13 + 2*0.83 + 3*0.14;
  const double l600g400 = 2*0.30 + 3*0.61 + 4*0.09;
  const double l800g600 = 3*0.35 + 4*0.62 + 5*0.03;
  const double l1000g800 = 4*0.43 + 5*0.49 + 6*0.06 + 7*0.02;
};

void printConnections(const std::vector<std::vector<int>>& nodeConnections);

void primAlgorithm(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
                   const int mBSId, std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                   std::vector<std::vector<Point_t>>& bsPairs);

void primAlgorithmSetLinksToGateway(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
                                    const int mBSId, const int numLinksToGateway,
                                    std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                                    std::vector<std::vector<Point_t>>& bsPairs);

void evaluateEstimateHopNumbers(std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet, const EstimatedHop& eHops);

Point_t proportionalPoint(const Point_t& p1, const Point_t& p2, double pro);

double dot(const Vector_t& v1, const Vector_t& v2);

Vector_t cross(const Vector_t& v1, const Vector_t& v2);

Vector_t normalize(const Vector_t& v);

std::vector<Building_t> getBuildingInfoFromFile(std::string dataBuildings, std::string dataBuildingVertices, SystemParameters& parameters);

/* Generate candidate base station locations randomly, and it also generates the locations of all relays located at the roof top corners. */
std::vector<Point_t> generateCandidateBaseStations(std::vector<Building_t>& buildingSet, std::vector<Point_t>& roofTopRelays, SystemParameters& parameters);

/* Select base stations based on the grid constraint. */
void selectBaseStationPerGrid(std::vector<Point_t>& bsSet, SystemParameters& parameters);

/* Collect all relays on the surfaces of buildings. */
std::vector<Point_t> collectAllRelays(const std::vector<Building_t>& buildings);

/* Select roof top relays based on the grid constraint. */
void selectRelayPerGrid(std::vector<Point_t>& relays, SystemParameters& parameters);

/* Explore the connectivity between each pair of nodes in the topology. */
std::vector<std::vector<int>> exploreConnectivity(const std::vector<Point_t>& nodes, const std::vector<Building_t>& buildings, const std::string& fileRelayNeighbors);

/* Evaluate the line-of-sight connectivity between a point s and other nodes in the topology. */
std::vector<int> searchNonBlockLink(const std::vector<Building_t>& buildings, const Point_t& s, const std::vector<Point_t>& nodes);

/* Test whether the line sd is intersect with any building in the building set. */
bool blockageTest(const std::vector<Building_t>& buildingSet, const Line_t& sd);

/* Read the connectivity information from file. */
void getRelayNeighborInfoFromFile(std::vector<std::vector<int>>& relayNeighborList, std::string dataRelayNeighbors);

/* Generate a number of base station pairs for the use in simulations. */
std::vector<std::vector<Point_t>> generateBaseStationPairs(const std::vector<Point_t>& bsSet, SystemParameters& parameters);

/* Add a new node to the graph, update the connectivity information. */
std::vector<std::vector<int>> addNodeToConnectivityList(const std::vector<std::vector<int>>& relayNeighborList,
                                                        const Point_t& newNode, const std::vector<Point_t>& oldNodes,
                                                        const std::vector<Building_t>& buildings);

/* The algorithm proposed in MASS paper. */
std::vector<std::vector<int>> findPathDecodeForward(const std::vector<std::vector<int>>& nodeNeighborList,
                                                    const std::vector<Point_t>& nodes, int addHop,
                                                    SystemParameters& parameters);

/* Find the optimal path between a pair of source and destination. */
std::vector<int> findPathDecodeForwardMaxHop(const std::vector<std::vector<int>>& nodeNeighborList,
                                             const std::vector<Point_t>& nodes, int maxHop,
                                             SystemParameters& parameters);

/* The recursive algorithm to find the path. */
std::vector<std::vector<int>> findNextHopNode(const std::vector<std::vector<int>>& nodeNeighborList,
                                              const std::vector<Point_t>& nodes, int maxHop, SystemParameters& parameters,
                                              int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput);

/* Find the optimal path between a pair of source and destination. */
void searchPathDecodeForwardMaxHop(Path_t& paths, const std::vector<Point_t>& nodes,
                                   const std::vector<std::vector<int>>& nodeNeighborList,
                                   const int& relayNum, SystemParameters& parameters);

/* Search the next hop along a path. */
void searchNextHopNode(Path_t& paths, const std::vector<int>& curPath, const std::vector<Point_t>& nodes,
                       const std::vector<std::vector<int>>& nodeNeighborList, const int& relayNum,
                       const int& preHopNum, const int& maxHopNum, const double& preHopCap,
                       const double& curPathThroughput, SystemParameters& parameters);

/* Detect the intra path interference when a new physical link is added on the right side of the path. */
bool intraPathInterferenceAddLink(const std::vector<int>& path, const int& lLId, const int& lRId,
                                  const std::vector<Point_t>& nodes,
                                  const std::vector<std::vector<int>>& nodeNeighborList,
                                  const SystemParameters& parameters);

/* Dijkstra algorithm to find the shortest path. */
std::vector<int> Dijkstra(const std::vector<std::vector<int>>& neighborList, const std::vector<Point_t>& nodes,
                          std::string pathFile, std::string type);

/* Estimate the link capacity based on Shannon Equation. */
double calculateLinkCapacity_Gbps(double linkLength_m, SystemParameters& parameters);

/* Estimate the link weight in AF scheme. */
double calculateWeight(double dist);

/* Check whether two paths interfere with each other. */
bool checkTwoPathsInterference(const std::vector<int>& path1, const std::vector<int>& path2,
                               const std::vector<Point_t>& sd1, const std::vector<Point_t>& sd2,
                               const std::vector<std::vector<int>>& nodeNeighborList,
                               const std::vector<Building_t>& buildings,
                               const std::vector<Point_t>& nodes, const SystemParameters& parameters);

#endif //FINALPROJECT_3DMODELING_H
