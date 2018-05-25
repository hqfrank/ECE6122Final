#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <memory>
#include <map>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "3DModeling.h"
#include "UtilityMethods.h"
#include "Plane_t.h"
#include "Building_t.h"
#include "SystemParameters.h"

using namespace std;

int main() {
  /*
   *  ***************************************
   *  ************ MAIN FUNCTION ************
   *  ***************************************
   */
  /*
   * ===============================================================================
   * Get current time, which is used to identify the name of the simulation results.
   * ===============================================================================
   */
  std::chrono::microseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );
  std::string strTime = std::to_string(ms.count()/1000);
//  cout << strTime << endl;
  /*
   * ===========================================
   * Set up simulation configuration parameters.
   * ===========================================
   */
  SystemParameters sysParams;
  sysParams.simStartTime = strTime;
  EstimatedHop eHops;
//  cout << "randomSeed in use is " + std::to_string(sysParams.randomSeed) << endl;
  /*
   * ======================
   * Files addresses in use
   * ======================
   */
  /* Building raw data file */
  std::string strDataBuildings = "../Data/Data_BuildingInfo_ATL.txt";
  /* File to store the vertices of each building */
  std::string strDataBuildingVertices = "../Data/Building_Vertices/Data_BuildingVertices_" + std::to_string(sysParams.randomSeed) +".txt";
  /* File to store the connectivity information of the relay nodes. */
  std::string dataRelayNeighbors = "../Data/Relay_Neighbors/Data_RelayNeighbors_"+ std::to_string(sysParams.randomSeed)
                                   + "_" + std::to_string(sysParams.maxNumRelaysInGrid)
                                   + "_" + std::to_string(sysParams.phyLinkDistMax_m)
                                   + "_" + sysParams.relayType + ".txt";
  /* File to store the time stamp of the simulation corresponding to each pair of source and destination base stations. */
  std::string strTimeStampFile = "../Data/Paths/" + strTime + ".txt";

  /*
   * ====================================
   * Construct all buildings in the area.
   * ====================================
   */
  std::vector<Building_t> buildingSet = getBuildingInfoFromFile(strDataBuildings, strDataBuildingVertices, sysParams);

  /*
   * ===================================================
   * Generate candidate base station locations randomly.
   * ===================================================
   */
  std::vector<Point_t> roofTopRelays;
  std::vector<Point_t> bsSet = generateCandidateBaseStations(buildingSet, roofTopRelays, sysParams);
//  cout << to_string(roofTopRelays.size()) << endl;
  /*
   * ==================================================
   * Select base stations based on the grid constraint.
   * ==================================================
   */
  selectBaseStationPerGrid(bsSet, sysParams);

  /*
   * ==================================================================
   * Approximately evaluate the hops between any pair of base stations.
   * ==================================================================
   */
  auto numBSs = bsSet.size();
  std::vector<std::vector<double>> eHopMap;
  evaluateEstimateHopNumbers(eHopMap, bsSet, eHops);
  std::vector<std::vector<int>> nodeConnections(numBSs, std::vector<int>());
  std::vector<std::vector<int>> treeConnections;
  std::vector<std::vector<Point_t>> bsPairs;
  primAlgorithm(eHopMap, bsSet, bsSet.size()/2, nodeConnections, treeConnections, bsPairs);
  printConnections(nodeConnections);

  /*
   * =========================================
   * Collect all candidate relays in the area.
   * =========================================
   */
  std::vector<Point_t> allRelays = collectAllRelays(buildingSet);
  /*
   * =======================================================================================
   * If roof top relays are in use, select roof top relays according to the grid constraint.
   * =======================================================================================
   */
  if (sysParams.relayType.compare("Top") == 0) {
    selectRelayPerGrid(roofTopRelays, sysParams);
  }
  /*
   * ==================================================================================================================
   * Check if connectivity information exists, if not, generate connectivity information; otherwise, read existed info.
   * ==================================================================================================================
   */
  std::ifstream fileConnect(dataRelayNeighbors);
  std::vector<std::vector<int>> relayNeighborList;
  if (fileConnect.good()){
    // read the relay neighborList
    getRelayNeighborInfoFromFile(relayNeighborList, dataRelayNeighbors);
  } else {
    relayNeighborList = exploreConnectivity(allRelays, buildingSet, dataRelayNeighbors);
  }

  /*
   * ==============================================================
   * Add source and destination bss to "nodes" and "neighbor list".
   * ==============================================================
   */
  std::vector<std::vector<int>> nodeNeighborList = relayNeighborList;
  std::vector<Point_t> allNodes = allRelays;
  for (auto bs : bsSet){
    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, bs, allNodes, buildingSet);
    allNodes.push_back(bs);
  }

  /*
   * =========================================================================
   * For each logical link in the tree/mesh, run the path selection alogrithm.
   * =========================================================================
   */
  int numRelays = allRelays.size();
  int extraHopNum = 2;
  std::vector<std::vector<int>> allPaths;
  for (int i = 0; i < treeConnections.size(); i++) {
    cout << "\n=========================================================\n";
    cout << "Path search for the " << i << "-th pair of base stations.\n";
    cout << "---------------------------------------------------------" << endl;
    int srcId = numRelays + treeConnections[i][0];
    int dstId = numRelays + treeConnections[i][1];
    cout << "Src: " << allNodes[srcId].toString() << endl;
    cout << "Dst: " << allNodes[dstId].toString() << endl;
    cout << "---------------------------------------------------------" << endl;
    int caseCount = 0;
    for (int j = 1; caseCount <= extraHopNum; j++) {
      Path_t pathList(srcId, dstId, j);
      searchPathDecodeForwardMaxHop(pathList, allNodes, nodeNeighborList, numRelays, sysParams);
      if (pathList.pathList.size() > 0) {
        caseCount++;
//        if (pathList.getSingleHopMaxThroughputId() >= 0) {
//          cout << "The single hop path is with throughput " << pathList.getSingleHopMaxThroughput() << " Gbps." << endl;
//        }
//        cout << "The best multihop path is with " << pathList.pathList[pathList.getMultiHopMaxThroughputId()].size()-1
//                                                  << " hops and throughput " << pathList.getMultiHopMaxThroughput()
//                                                                             << " Gbps." << endl;
        if (j == 1) {
          assert(pathList.getSingleHopMaxThroughputId() >= 0);
          allPaths.push_back(pathList.pathList[pathList.getSingleHopMaxThroughputId()]);
        } else if (pathList.getMultiHopMaxThroughputId() >= 0){
          allPaths.push_back(pathList.pathList[pathList.getMultiHopMaxThroughputId()]);
        } else {
          allPaths.push_back(pathList.pathList[pathList.getSingleHopMaxThroughputId()]);
        }
      }
      cout << "There are " << pathList.pathList.size() << " paths with no more than " << j << " hops being found.\n";
      cout << "---------------------------------------------------------" << endl;

    }

  }

  cout << allPaths.size() << endl;
  assert(allPaths.size() == treeConnections.size()*(extraHopNum+1));

  std::map<int, std::vector<int>> nodeCheckMinHop;
  std::map<int, std::vector<int>>::iterator it;
  for (int i = 0; i < treeConnections.size(); i++) {
    for (int j = 1; j < allPaths[i*3].size() - 1; ++j){
      it = nodeCheckMinHop.find(allPaths[i*3][j]);
      if (it != nodeCheckMinHop.end()){
        nodeCheckMinHop.at(allPaths[i*3][j]).push_back(i);
      } else {
        std::vector<int> newPair;
        newPair.push_back(i);
        nodeCheckMinHop.insert(std::pair<int, std::vector<int>>(allPaths[i*3][j], newPair));
      }
    }
  }

  cout << "========== Check the min hop case ===========" << endl;
  for (it = nodeCheckMinHop.begin(); it != nodeCheckMinHop.end(); ++it) {
    if (it->second.size() > 1){
      cout << "-----------------------------------------------------------" << endl;
      cout << "Relay No. " << it->first << " has been selected by " << it->second.size() << " paths." << endl;
      int numPath = it->second.size();
      std::vector<int> pathIDs = it->second;
      for (int j = 0; j < numPath-1; j++){
        std::vector<int> path1 = allPaths[pathIDs[j]*3];
        Point_t src1 = bsPairs[pathIDs[j]][0];
        Point_t dst1 = bsPairs[pathIDs[j]][1];
        for (int k = j+1; k < numPath; k++) {
          std::vector<int> path2 = allPaths[pathIDs[k]*3];
          Point_t src2 = bsPairs[pathIDs[k]][0];
          Point_t dst2 = bsPairs[pathIDs[k]][1];
          std::vector<double> distance;
          distance.push_back(src1.distanceTo(src2));
          distance.push_back(src1.distanceTo(dst2));
          distance.push_back(dst1.distanceTo(src2));
          distance.push_back(dst1.distanceTo(dst2));
          double minDistance = GetMin(distance);
          cout << "Path " << pathIDs[j] << ": src - " << src1.toString() << ", dst - " << dst1.toString() << "\n"
               << "Path " << pathIDs[k] << ": src - " << src2.toString() << ", dst - " << dst2.toString() << "\n"
               << "Minimum distance between end points: " << minDistance << endl;
//          cout << minDistance << src1.distanceTo(src2) << "\t" << src1.distanceTo(dst2) << "\t"
//               << dst1.distanceTo(src2) << "\t" << dst1.distanceTo(dst2) << endl;
        }
      }
    }
  }

  // Check interference cases:
  int countIntPairs = 0;
  for (int i = 0; i < treeConnections.size()-1; ++i) {
    std::vector<int> path1 = allPaths[i*3];
    std::vector<Point_t> sd1 = bsPairs[i];
    for (int j = i+1; j < treeConnections.size(); ++j) {
      std::vector<int> path2 = allPaths[j*3];
      std::vector<Point_t> sd2 = bsPairs[j];
      bool intTest = checkTwoPathsInterference(path1, path2, sd1, sd2, nodeNeighborList, buildingSet, allNodes, sysParams);
      if(intTest) {
        cout << "Path " << i << " and Path " << j << " interfere with each other." << endl;
        countIntPairs++;
      }
    }
  }
  cout << "In total, there are " << countIntPairs << " pairs of paths interfere with each other." << endl;
//  fileOutTimeStamp.close();



//  /*
//   * ==============================================================================================================
//   * Select 100 pairs of source and destination base stations, the distance between which is in a certain category.
//   * Only when there is no tree or mesh topology existing in the network.
//   * ==============================================================================================================
//   */
//  if (bsPairs.empty()) {
//    bsPairs = generateBaseStationPairs(bsSet, sysParams);
//  }
//
//  /*
//   * ==============================================================================================
//   * For each pair of source and destination base stations, run the optimal path finding algorithm.
//   * ==============================================================================================
//   */
//  auto numBSPairs = bsPairs.size();
//  /* Open file to write the time stamp of the simulation on each pair of source, destination pairs. */
//  std::ofstream fileOutTimeStamp;
//  fileOutTimeStamp.open(strTimeStampFile, std::ios_base::app);
//  if (!fileOutTimeStamp.is_open()) {
//    cerr << "Error!!!The file to store time stamp is not open!!" << endl;
//    exit(errno);
//  }
//  int extraHop = 0;
//  std::vector<std::vector<int>> minHopPaths;
//  std::vector<std::vector<int>> minHopPlusOnePaths;
//  std::vector<std::vector<int>> minHopPlusTwoPaths;
//  std::map<int, std::vector<int>> nodeCheckMinHop;
//  std::map<int, std::vector<int>> nodeCheckMinHopPlusOne;
//  std::map<int, std::vector<int>> nodeCheckMinHopPlusTwo;
//  std::map<int, std::vector<int>>::iterator it;
//  for (int i = 0; i < numBSPairs; i++) {
//    /* Get current time as time stamp of the i th loop. */
//    std::chrono::microseconds curMs = std::chrono::duration_cast< std::chrono::milliseconds >(
//      std::chrono::system_clock::now().time_since_epoch()
//    );
//    std::string strTimeI = std::to_string(curMs.count()/1000);
//    /* Write the time stamp to the file which stores all time stamps. */
//    fileOutTimeStamp << strTimeI << "\n";
//    /* Print out the current source and destination */
//    Point_t srcTemp = bsPairs[i][0];
//    Point_t dstTemp = bsPairs[i][1];
//    cout << "=================== The " + std::to_string(i) + "-th pair s-d ==================" << endl;
//    cout << "source:      " + srcTemp.toString() << endl;
//    cout << "destination: " + dstTemp.toString() << endl;
//    /* Add source node to the graph. */
//    std::vector<std::vector<int>> nodeNeighborList = addNodeToConnectivityList(relayNeighborList, srcTemp, allRelays, buildingSet);
//    std::vector<Point_t> allNodes = allRelays;
//    allNodes.push_back(srcTemp);
//    /* Add destination node to the graph. */
//    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, dstTemp, allNodes, buildingSet);
//    allNodes.push_back(dstTemp);
//
//    std::vector<std::vector<int>> allPaths = findPathDecodeForward(nodeNeighborList, allNodes, extraHop, sysParams);
//    minHopPaths.push_back(allPaths.at(0));
//    for (int j = 1; j < allPaths.at(0).size() - 1; ++j){
//      it = nodeCheckMinHop.find(allPaths.at(0).at(j));
//      if (it != nodeCheckMinHop.end()){
//        nodeCheckMinHop.at(allPaths.at(0).at(j)).push_back(i);
//      } else {
//        std::vector<int> newPair;
//        newPair.push_back(i);
//        nodeCheckMinHop.insert(std::pair<int, std::vector<int>>(allPaths.at(0).at(j), newPair));
//      }
//    }
//    if (extraHop > 0) {
//      minHopPlusOnePaths.push_back(allPaths.at(1));
//      for (int j = 1; j < allPaths.at(1).size() - 1; ++j){
//        it = nodeCheckMinHopPlusOne.find(allPaths.at(1).at(j));
//        if (it != nodeCheckMinHopPlusOne.end()){
//          nodeCheckMinHopPlusOne.at(allPaths.at(1).at(j)).push_back(i);
//        } else {
//          std::vector<int> newPair;
//          newPair.push_back(i);
//          nodeCheckMinHopPlusOne.insert(std::pair<int, std::vector<int>>(allPaths.at(1).at(j), newPair));
//        }
//      }
//      if (extraHop > 1) {
//        minHopPlusTwoPaths.push_back(allPaths.at(2));
//        for (int j = 1; j < allPaths.at(2).size() - 1; ++j){
//          it = nodeCheckMinHopPlusTwo.find(allPaths.at(2).at(j));
//          if (it != nodeCheckMinHopPlusTwo.end()){
//            nodeCheckMinHopPlusTwo.at(allPaths.at(2).at(j)).push_back(i);
//          } else {
//            std::vector<int> newPair;
//            newPair.push_back(i);
//            nodeCheckMinHopPlusTwo.insert(std::pair<int, std::vector<int>>(allPaths.at(2).at(j), newPair));
//          }
//        }
//      }
//    }
//  }
////  cout << minHopPaths.size() << "\t" << minHopPlusOnePaths.size() << "\t" << minHopPlusTwoPaths.size() << endl;
//  cout << "========== Check the min hop case ===========" << endl;
//  for (it = nodeCheckMinHop.begin(); it != nodeCheckMinHop.end(); ++it) {
//    if (it->second.size() > 1){
//      cout << "-----------------------------------------------------------" << endl;
//      cout << "Relay No. " << it->first << " has been selected by " << it->second.size() << " paths." << endl;
//      int numPath = it->second.size();
//      std::vector<int> pathIDs = it->second;
//      for (int j = 0; j < numPath-1; j++){
//        std::vector<int> path1 = minHopPaths.at(pathIDs.at(j));
//        Point_t src1 = bsPairs[pathIDs.at(j)][0];
//        Point_t dst1 = bsPairs[pathIDs.at(j)][1];
//        for (int k = j+1; k < numPath; k++) {
//          std::vector<int> path2 = minHopPaths.at(pathIDs.at(k));
//          Point_t src2 = bsPairs[pathIDs.at(k)][0];
//          Point_t dst2 = bsPairs[pathIDs.at(k)][1];
//          std::vector<double> distance;
//          distance.push_back(src1.distanceTo(src2));
//          distance.push_back(src1.distanceTo(dst2));
//          distance.push_back(dst1.distanceTo(src2));
//          distance.push_back(dst1.distanceTo(dst2));
//          double minDistance = GetMin(distance);
//          cout << "Path " << pathIDs[j] << ": src - " << src1.toString() << ", dst - " << dst1.toString() << "\n"
//               << "Path " << pathIDs[k] << ": src - " << src2.toString() << ", dst - " << dst2.toString() << "\n"
//               << "Minimum distance between end points: " << minDistance << endl;
////          cout << minDistance << src1.distanceTo(src2) << "\t" << src1.distanceTo(dst2) << "\t"
////               << dst1.distanceTo(src2) << "\t" << dst1.distanceTo(dst2) << endl;
//        }
//      }
//    }
//  }
//
//
//  cout << "========== Check the min hop plus one case ===========" << endl;
//  for (it = nodeCheckMinHopPlusOne.begin(); it != nodeCheckMinHopPlusOne.end(); ++it) {
//    if (it->second.size() > 1){
//      cout << "-----------------------------------------------------------" << endl;
//      cout << "Relay No. " << it->first << " has been selected by " << it->second.size() << " paths." << endl;
//      int numPath = it->second.size();
//      std::vector<int> pathIDs = it->second;
//      for (int j = 0; j < numPath-1; j++){
//        std::vector<int> path1 = minHopPlusOnePaths.at(pathIDs.at(j));
//        Point_t src1 = bsPairs[pathIDs.at(j)][0];
//        Point_t dst1 = bsPairs[pathIDs.at(j)][1];
//        for (int k = j+1; k < numPath; k++) {
//          std::vector<int> path2 = minHopPlusOnePaths.at(pathIDs.at(k));
//          Point_t src2 = bsPairs[pathIDs.at(k)][0];
//          Point_t dst2 = bsPairs[pathIDs.at(k)][1];
//          std::vector<double_t> distance;
//          distance.push_back(src1.distanceTo(src2));
//          distance.push_back(src1.distanceTo(dst2));
//          distance.push_back(dst1.distanceTo(src2));
//          distance.push_back(dst1.distanceTo(dst2));
//          double minDistance = GetMin(distance);
//          cout << "Path " << pathIDs[j] << ": src - " << src1.toString() << ", dst - " << dst1.toString() << "\n"
//               << "Path " << pathIDs[k] << ": src - " << src2.toString() << ", dst - " << dst2.toString() << "\n"
//               << "Minimum distance between end points: " << minDistance << endl;
////          cout << minDistance << src1.distanceTo(src2) << "\t" << src1.distanceTo(dst2) << "\t"
////               << dst1.distanceTo(src2) << "\t" << dst1.distanceTo(dst2) << endl;
//        }
//      }
//    }
//  }
//
//  cout << "========== Check the min hop plus two case ===========" << endl;
//  for (it = nodeCheckMinHopPlusTwo.begin(); it != nodeCheckMinHopPlusTwo.end(); ++it) {
//    if (it->second.size() > 1){
//      cout << "-----------------------------------------------------------" << endl;
//      if (it->first > 1132) {
//        cout << "Error!" << endl;
//      }
//      cout << "Relay No. " << it->first << " has been selected by " << it->second.size() << " paths." << endl;
//      int numPath = it->second.size();
//      std::vector<int> pathIDs = it->second;
//      for (int j = 0; j < numPath-1; j++){
//        std::vector<int> path1 = minHopPlusTwoPaths.at(pathIDs.at(j));
//        Point_t src1 = bsPairs[pathIDs.at(j)][0];
//        Point_t dst1 = bsPairs[pathIDs.at(j)][1];
//        for (int k = j+1; k < numPath; k++) {
//          std::vector<int> path2 = minHopPlusTwoPaths.at(pathIDs.at(k));
//          Point_t src2 = bsPairs[pathIDs.at(k)][0];
//          Point_t dst2 = bsPairs[pathIDs.at(k)][1];
//          std::vector<double> distance;
//          distance.push_back(src1.distanceTo(src2));
//          distance.push_back(src1.distanceTo(dst2));
//          distance.push_back(dst1.distanceTo(src2));
//          distance.push_back(dst1.distanceTo(dst2));
//          double minDistance = GetMin(distance);
//          cout << "Path " << pathIDs[j] << ": src - " << src1.toString() << ", dst - " << dst1.toString() << "\n"
//               << "Path " << pathIDs[k] << ": src - " << src2.toString() << ", dst - " << dst2.toString() << "\n"
//               << "Minimum distance between end points: " << minDistance << endl;
////          cout << minDistance << src1.distanceTo(src2) << "\t" << src1.distanceTo(dst2) << "\t"
////               << dst1.distanceTo(src2) << "\t" << dst1.distanceTo(dst2) << endl;
//        }
//      }
//    }
//  }
//
//  // Check interference cases:
//  int countIntPairs = 0;
//  for (int i = 0; i < minHopPaths.size()-1; ++i) {
//    std::vector<int> path1 = minHopPaths[i];
//    std::vector<Point_t> sd1 = bsPairs[i];
//    for (int j = i+1; j < minHopPaths.size(); ++j) {
//      std::vector<int> path2 = minHopPaths[j];
//      std::vector<Point_t> sd2 = bsPairs[j];
//      bool intTest = checkTwoPathsInterference(path1, path2, sd1, sd2, relayNeighborList, buildingSet, allRelays, sysParams);
//      if(intTest) {
//        cout << "Path " << i << " and Path " << j << " interfere with each other." << endl;
//        countIntPairs++;
//      }
//    }
//  }
//  cout << "In total, there are " << countIntPairs << " pairs of paths interfere with each other." << endl;
//  fileOutTimeStamp.close();

  cout << "==================================" << endl;
  cout << "This is the end of the simulation." << endl;
  cout << "==================================" << endl;






  return 0;
}