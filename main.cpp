#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include <memory>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "3DModeling.h"
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
//  cout << to_string(bsSet.size()) << endl;
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
   * ==============================================================================================================
   * Select 100 pairs of source and destination base stations, the distance between which is in a certain category.
   * ==============================================================================================================
   */
  std::vector<Point_t> bsPairs = generateBaseStationPairs(bsSet, sysParams);

  /*
   * ==============================================================================================
   * For each pair of source and destination base stations, run the optimal path finding algorithm.
   * ==============================================================================================
   */
  std::ofstream fileOutTimeStamp;
  fileOutTimeStamp.open(strTimeStampFile, std::ios_base::app);
  if (!fileOutTimeStamp.is_open()) {
    cerr << "Error!!!The file to store time stamp is not open!!" << endl;
    exit(errno);
  }
  for (int i = 0; i < sysParams.numBSPairs; i++) {
    /* Get current time as time stamp of the i th loop. */
    std::chrono::microseconds curMs = std::chrono::duration_cast< std::chrono::milliseconds >(
      std::chrono::system_clock::now().time_since_epoch()
    );
    std::string strTimeI = std::to_string(curMs.count()/1000);
    /* Write the time stamp to the file which stores all time stamps. */
    fileOutTimeStamp << strTimeI << "\n";
    /* Print out the current source and destination */
    Point_t srcTemp = bsPairs.at(i*2);
    Point_t dstTemp = bsPairs.at(i*2 + 1);
    cout << "=================== The " + std::to_string(i) + "-th pair s-d ==================" << endl;
    cout << "source:      " + srcTemp.toString() << endl;
    cout << "destination: " + dstTemp.toString() << endl;
    /* Add source node to the graph. */
    std::vector<std::vector<int>> nodeNeighborList = addNodeToConnectivityList(relayNeighborList, srcTemp, allRelays, buildingSet);
    std::vector<Point_t> allNodes = allRelays;
    allNodes.push_back(srcTemp);
    /* Add destination node to the graph. */
    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, dstTemp, allNodes, buildingSet);
    allNodes.push_back(dstTemp);

    std::vector<std::vector<int>> allPaths = findPathDecodeForward(nodeNeighborList, allNodes, 2, sysParams);

  }
  fileOutTimeStamp.close();

  cout << "==================================" << endl;
  cout << "This is the end of the simulation." << endl;
  cout << "==================================" << endl;






  return 0;
}