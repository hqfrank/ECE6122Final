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
#include <map>
#include <unordered_set>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "Building_t.h"
#include "SystemParameters.h"
#include "Plane_t.h"
#include "Path_t.h"
#include "PhyLink.h"
#include "UtilityMethods.h"

void calculatePathThroughputPhyIntModel(const std::string& thoughputFile, const std::string& pathFile,
                                        const std::string& capacityFile, const std::vector<Point_t>& allNodes,
                                        const std::vector<std::vector<int>>& nodeNeighborList,
                                        const SystemParameters& parameters);

double calculateMutualInterferenceTwoNodes(const int nId, const Point_t& n, const Vector_t& dirN, const bool nIsBS,
                                           const int nIntId, const Point_t& nInt, const Vector_t& dirNInt,
                                           const bool nIntIsBS, const SystemParameters& parameters,
                                           const double pt_dBm,
                                           const std::vector<int>& nNeighbors);

void calculateMutualInterferenceTwoPhyLinks(double& intAtN1, double& intAtN2,
                                            const PhyLink& pLink, const PhyLink& pLinkInt,
                                            const bool pLN1IsBs, const bool pLIntN2IsBs,
                                            const std::vector<int>& pLinkN1Neighbors,
                                            const std::vector<int>& pLinkN2Neighbors,
                                            const SystemParameters& parameters, const double pt_dBm);

class EstimatedHop {
public:
  const double l200g20 = 1*0.48+2*0.51+3*0.01;
  const double l400g200 = 1*0.13 + 2*0.83 + 3*0.14;
  const double l600g400 = 2*0.30 + 3*0.61 + 4*0.09;
  const double l800g600 = 3*0.35 + 4*0.62 + 5*0.03;
  const double l1000g800 = 4*0.43 + 5*0.49 + 6*0.06 + 7*0.02;
};

void getBuildingInfoFromFile(std::vector<Building_t>& buildings, std::string dataBuildings,
                             std::string dataBuildingVertices, SystemParameters& parameters);

/* Collect all relays on the surfaces of buildings. */
void collectAllRelays(std::vector<Point_t>& allRelays, const std::vector<Building_t>& buildings,
                      const std::string& fileDataRelays);

void writePathsToFile(const std::vector<std::vector<int>>& allPaths, const std::vector<int>& sequence,
                      const std::vector<Point_t>& allNodes, SystemParameters& parameters,
                      std::string& dataPath);

void writeVectorDataToFile(const string& filename, const std::vector<std::vector<int>>& data);

void treeTopologyMeshAtlanta(const int mBSPos[2], std::vector<std::vector<int>>& bsGridMap,
                             const std::vector<std::vector<int>>& bsLocation,
                             const std::vector<Point_t>& bsSet,
                             std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                             std::vector<std::vector<Point_t>>& bsPairs, std::vector<double>& demandLink,
                             const SystemParameters& parameters);

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



/* Generate candidate base station locations randomly, and it also generates the locations of all relays located at the roof top corners. */
std::vector<Point_t> generateCandidateBaseStations(std::vector<Building_t>& buildingSet, std::vector<Point_t>& roofTopRelays, SystemParameters& parameters);

/* Select base stations based on the grid constraint. */
void selectBaseStationPerGrid(std::vector<Point_t>& bsSet, std::vector<std::vector<int>>& bsGridMap,
                              std::vector<std::vector<int>>& bsLocation,
                              std::vector<std::vector<int>>& numRelaysInGrid,
                              std::string& dataBSs, bool write, SystemParameters& parameters);

void writeBSsLoactionToFile(const std::vector<std::vector<int>>& bsLocations, const std::string& dataBSsGrid);

void writeSpaceDiversityToFile(int randomSeed, int numRelays, int spaceDiversity, const std::string& dataSpaceDiversity);

/* Read node information from file. */
void readNodeInfoFromFile(std::vector<Point_t>& nodes, const std::string& fileDataNodes, std::string& type);

/* Select roof top relays based on the grid constraint. */
void selectRelayPerGrid(std::vector<Point_t>& relays, SystemParameters& parameters);

/* Count the number of relays per grid */
void countRelaysPerGrid(std::vector<Point_t>& relays, std::vector<std::vector<int>>& numRelaysInGrid, SystemParameters& parameters);

/* Explore the connectivity between each pair of nodes in the topology. */
void exploreConnectivity(std::vector<std::vector<int>>& neighborList,
                         const std::vector<Point_t>& nodes, const std::vector<Building_t>& buildings,
                         const std::string& fileRelayNeighbors);

/* Collect physical links within selected area. */
void collectPhysicalLinks(std::vector<std::vector<int>>& phyLinkSet, const std::vector<std::vector<int>>& neighborList,
                          const std::vector<Point_t>& nodes, const vector<int>& selectedGrids,
                          const SystemParameters& parameters, std::string& dataPhyLinks, bool& write);

/* Collect consective link pairs upon selected physical links. */
void collectConsecutiveLinkPairs(std::vector<int>& consecLinkPairSet,
                                 const std::vector<std::vector<int>>& phyLinkSet,
                                 std::string& dataConsecLinkPairs, bool& write);

/* Collect first/last hop physical links of each connection. */
void collectFirstLastHopCandidatePhyLinks(std::vector<std::vector<int>>& firstHopSet,
                                          std::vector<std::vector<int>>& lastHopSet,
                                          const std::vector<std::vector<int>>& phyLinkSet,
                                          const std::vector<std::vector<int>>& connectionList,
                                          int& numRelays, bool& write, std::string& dataFirstHop,
                                          std::string& dataLastHop);

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

/* Find the optimal path between a pair of source and destination. */
void searchPathDecodeForwardMaxHop(Path_t& paths, const std::vector<Point_t>& nodes,
                                   const std::vector<std::vector<int>>& nodeNeighborList,
                                   const int& relayNum, SystemParameters& parameters,
                                   const std::map<int, std::vector<Vector_t>>& phyLinksAtBSs,
                                   const std::map<int, Vector_t>& phyLinks,
                                   const std::vector<int>& selectedRelays);

/* Search the next hop along a path. */
void searchNextHopNode(Path_t& paths, const std::vector<int>& curPath, const std::vector<Point_t>& nodes,
                       const std::vector<std::vector<int>>& nodeNeighborList, const int& relayNum,
                       const int& preHopNum, const int& maxHopNum, const double& preHopCap,
                       const double& curPathThroughput, SystemParameters& parameters,
                       const std::map<int, std::vector<Vector_t>>& phyLinksAtBSs,
                       const std::map<int, Vector_t>& phyLinks,
                       const std::vector<int>& selectedRelays);

/* Detect the intra path interference when a new physical link is added on the right side of the path. */
bool intraPathInterferenceAddLink(const std::vector<int>& path, const int& lLId, const int& lRId,
                                  const std::vector<Point_t>& nodes,
                                  const std::vector<std::vector<int>>& nodeNeighborList,
                                  const SystemParameters& parameters);

/* Dijkstra algorithm to find the shortest path. */
std::vector<int> Dijkstra(const std::vector<std::vector<int>>& neighborList, const std::vector<Point_t>& nodes,
                          int srcId, std::string pathFile, std::string type);

/* Estimate the link capacity based on Shannon Equation. */
double calculateLinkCapacity_Gbps(double linkLength_m, SystemParameters& parameters);

/* Estimate the link weight in AF scheme. */
double calculateWeight(double dist);

bool checkInterPathInterference(const int s1i, const int d1i, const std::map<int, Vector_t>& phyLinks,
                                const std::vector<Point_t>& nodes,
                                const std::vector<std::vector<int>>& nodeNeighborList,
                                const SystemParameters& parameters);

/* Check whether two paths interfere with each other. */
bool checkTwoPathsInterference(const std::vector<int>& path1, const std::vector<int>& path2,
                               const std::vector<Point_t>& sd1, const std::vector<Point_t>& sd2,
                               const std::vector<std::vector<int>>& nodeNeighborList,
                               const std::vector<Building_t>& buildings,
                               const std::vector<Point_t>& nodes, const SystemParameters& parameters);

void recordPhysicalLinksInAPath(std::map<int, Vector_t>& allPhysicalLinks, const std::vector<int>& path,
                                const std::vector<Point_t>& nodes, const SystemParameters& parameters);

void recordRelaysInAPath(std::vector<int>& allRelaysSelected, const std::vector<int>& path,
                         const std::vector<Point_t>& nodes, const SystemParameters& parameters);

void collectPhyLinksAtBSs(std::map<int, std::vector<Vector_t>>& phyLinksAtBSs, const std::vector<int>& path,
                          const std::vector<Point_t>& nodes);

int evaluateSpaceDiversityAtNode(const int nodeId, const std::vector<Point_t>& nodes,
                                 std::vector<int>& maxSDNodeList,
                                 const std::vector<std::vector<int>>& nodeNeighborList,
                                 const SystemParameters& parameters);

void BronKerboschPivoting(const std::vector<std::vector<int>>& graph, const std::vector<int>& R, std::vector<int> P,
                          std::vector<int> X, std::vector<std::vector<int>>& allMaximalCliques, int& maxDegree);

bool checkTwoPhysicalLinksInterference(const std::vector<int>& phyLink1, const std::vector<int>& phyLink2,
                                       int& numRelays, const std::vector<Point_t>& nodes,
                                       const SystemParameters& parameters);

void collectMutualInterferenceInfo(std::vector<std::vector<int>>& mutualInterferenceIndicator,
                                   const std::vector<std::vector<int>>& phyLinkSet, int& numRelays,
                                   const std::vector<Point_t>& nodes, const SystemParameters& parameters,
                                   std::string& dataMutualInterference, bool& write);

void writeTopologyToFile(std::string& dataTopology, const std::vector<std::vector<int>>& connections, int& numRelays);

///* The algorithm proposed in MASS paper. */
//std::vector<std::vector<int>> findPathDecodeForward(const std::vector<std::vector<int>>& nodeNeighborList,
//                                                    const std::vector<Point_t>& nodes, int addHop,
//                                                    SystemParameters& parameters);

///* Find the optimal path between a pair of source and destination. */
//std::vector<int> findPathDecodeForwardMaxHop(const std::vector<std::vector<int>>& nodeNeighborList,
//                                             const std::vector<Point_t>& nodes, int maxHop,
//                                             SystemParameters& parameters);

///* The recursive algorithm to find the path. */
//std::vector<std::vector<int>> findNextHopNode(const std::vector<std::vector<int>>& nodeNeighborList,
//                                              const std::vector<Point_t>& nodes, int maxHop, SystemParameters& parameters,
//                                              int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput);

#endif //FINALPROJECT_3DMODELING_H
