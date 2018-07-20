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
   * ===================================================================================
   *   Get current time, which is used to identify the name of the simulation results.
   * ===================================================================================
   */
  std::chrono::microseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );
  std::string strTime = std::to_string(ms.count()/1000);  // "strTime" is the current time in string type.

  /*
   * ===============================================
   *   Set up simulation configuration parameters.
   * ===============================================
   */
  SystemParameters sysParams;        // Create the system parameter object.
  sysParams.simStartTime = strTime;  // Update the simulation start time using "strTime".
  EstimatedHop eHops;                // Create the variable which records the distance-estimated hop number mapping.

  /*
   * ==========================
   *   Files addresses in use
   * ==========================
   */
  /* Building raw data file (not change) */
  std::string strDataBuildings = "../Data/Data_BuildingInfo_ATL.txt";
  /* File to store the vertices of each building (not change)*/
  std::string strDataBuildingVertices = "../Data/Building_Vertices/Data_BuildingVertices.txt";

  /* File to store the time stamp of the simulation corresponding to each pair of source and destination base stations. */
  std::string strTimeStampFile = "../Data/Paths/" + strTime + ".txt";

  for (int rnd = 701; rnd < 800; rnd++) {
      sysParams.randomSeed = rnd;
      /*
       * ========================================
       *   Construct all buildings in the area.
       * ========================================
       */
      std::vector<Building_t> buildingSet = getBuildingInfoFromFile(strDataBuildings, strDataBuildingVertices,
                                                                    sysParams);

      /*
       * =============================================
       *   Collect all candidate relays in the area.
       * =============================================
       */
      /* File to store the relay nodes. */
      std::string dataRelays = "../Data/Relays/Data_Relays_" + sysParams.relayType
                               + "_" + std::to_string(sysParams.randomSeed)
                               + "_" + std::to_string(sysParams.densityRelayOnBuilding)
                               + "_" + std::to_string(sysParams.minNumRelaysPerFace) + ".txt";
      std::ifstream fileRelays(dataRelays);
      std::vector<Point_t> allRelays;
      if (fileRelays.good()) {
          // read the relays
          std::string type = "relay";
          readNodeInfoFromFile(allRelays, dataRelays, type);
      } else {
          allRelays = collectAllRelays(buildingSet, dataRelays);
      }
      int numRelays = allRelays.size();
      std::vector<std::vector<int>> numRelaysInGrid;
      countRelaysPerGrid(allRelays, numRelaysInGrid, sysParams);

      /*
       * =======================================================
       *   Generate candidate base station locations randomly.
       * =======================================================
       */
      /* File to store the selected base stations. */
      std::string dataBSs = "../Data/Base_Stations/Data_BSSet_" + std::to_string(sysParams.randomSeed)
                            + "_" + std::to_string(numRelays)
                            + "_" + std::to_string(sysParams.gridSize_m)
                            + "_" + std::to_string(sysParams.minRelayNumInGrid) + ".txt";
      std::string dataBSInGrid = "../Data/Base_Stations_Grid/Data_BSInGrid_" + std::to_string(sysParams.randomSeed)
                                 + "_" + std::to_string(numRelays)
                                 + "_" + std::to_string((int) round(sysParams.gridSize_m))
                                 + "_" + std::to_string(sysParams.minRelayNumInGrid) + ".txt";
      std::vector<Point_t> roofTopRelays;  // An Point_t vector to store all relays on the roof top.
      std::vector<Point_t> bsSet;
      std::ifstream fileBSs(dataBSs);
      if (fileBSs.good()) {
          // read the selected BSs
          std::string type = "base station";
          readNodeInfoFromFile(bsSet, dataBSs, type);
      } else {
          bsSet = generateCandidateBaseStations(buildingSet, roofTopRelays, sysParams);
      }
      /* Select base stations based on the grid. Each grid has at most 1 base station.  */
      std::vector<std::vector<int>> bsGridMap;   // Stores the index of base station (i.e. 'i' in bsSet[i]) in each grid.
      std::vector<std::vector<int>> bsLocation;  // Stores the indices of row and column of the grid where each base station sits.
      selectBaseStationPerGrid(bsSet, bsGridMap, bsLocation, numRelaysInGrid, dataBSs, !fileBSs.good(), sysParams);
      std::ifstream fileBSsGrid(dataBSInGrid);
      if (!fileBSsGrid.good()) {
          writeBSsLoactionToFile(bsLocation, dataBSInGrid);
      }
      int numBSs = bsSet.size();
      std::vector<Point_t> allNodes = allRelays;
      allNodes.insert(allNodes.end(), bsSet.begin(), bsSet.end());
      cout << "(*) There are " << allNodes.size() << " wireless nodes in the area." << endl;

      /*
       * ===============================================================
       *   Evaluate the connectivity between different wireless nodes.
       * ===============================================================
       */
      /* File to store the node neighboring information. */
      std::string dataNodeNeighbors =
              "../Data/Node_Neighbors/Data_NodeNeighbors_" + std::to_string(sysParams.randomSeed)
              + "_" + std::to_string(numRelays)
              + "_" + std::to_string(numBSs) + ".txt";
      std::ifstream fileConnect(dataNodeNeighbors);
      std::vector<std::vector<int>> nodeNeighborList;
      if (fileConnect.good()) {
          // read the relay neighborList
          getRelayNeighborInfoFromFile(nodeNeighborList, dataNodeNeighbors);
          assert(nodeNeighborList.size() == allNodes.size());
      } else {
          exploreConnectivity(nodeNeighborList, allNodes, buildingSet, dataNodeNeighbors);
      }

      /* Get the line-of-sight neighboring information of all base stations. */
      std::string dataBSNeighbors = "../Data/BS_Neighbors/Data_BSNeighbors_" + std::to_string(sysParams.randomSeed)
                                    + "_" + std::to_string(numBSs)
                                    + ".txt";
      std::ifstream fileBSConnect(dataBSNeighbors);
      std::vector<std::vector<int>> bsNeighborList;
      if (fileBSConnect.good()) {
          getRelayNeighborInfoFromFile(bsNeighborList, dataBSNeighbors);
      } else {
          exploreConnectivity(bsNeighborList, bsSet, buildingSet, dataBSNeighbors);
      }

      /*
       * =======================================
       *   Select the macro-cell base station.
       * =======================================
       */
      int mBSPos[2] = {4, 5};
      cout << "The macro-cell base station is in grid row " << mBSPos[0] << ", column " << mBSPos[1] << endl;
      /* Find the space diversity of macro cell base station. */
      std::vector<int> maxSDNodeList;
      int mBSSD = evaluateSpaceDiversityAtNode(allRelays.size() + bsGridMap[mBSPos[0]][mBSPos[1]], allNodes,
                                               maxSDNodeList, nodeNeighborList, sysParams);
      cout << "The space diversity at macro cell base station is: " << mBSSD << endl;

      /*
       * ======================================================
       *   Evaluate the LoS multi-hop paths from mBS to sBSs.
       * ======================================================
       */
      int mBSId = bsGridMap[mBSPos[0]][mBSPos[1]];
      std::vector<int> tempmBSPath = Dijkstra(bsNeighborList, bsSet, mBSId, "../Data/Paths/mBS2BS.txt", "distance");

      /*
       * ==================================================
       *   Collect physical links in the interested area.
       * ==================================================
       */
//      std::vector<std::vector<int>> phyLinkSet;
//      std::vector<int> selectedGrids = {31, 32, 40, 41};
//      std::string dataPhyLinks = "../Data/Physical_Links/Data_PhyLinks_" + std::to_string(sysParams.randomSeed)
//                                 + "_" + std::to_string(numRelays)
//                                 + "_" + std::to_string(numBSs) + "_" + std::to_string(selectedGrids.size())
//                                 + "_" + std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180)) +
//                                 ".txt";
//      std::ifstream filePhyLinks(dataPhyLinks);
//      bool writePhyLinks = false;
//      if (!filePhyLinks.good()) {
//          writePhyLinks = true;
//      }
//      collectPhysicalLinks(phyLinkSet, nodeNeighborList, allNodes, selectedGrids, sysParams, dataPhyLinks,
//                           writePhyLinks);

      /*
       * ===================================================================
       *   Collect consecutive link pairs upon selected physical link set.
       * ===================================================================
       */
      /* File to store the consecutive link pairs info. */
//      std::string dataConsecLinkPairs = "../Data/Consecutive_Link_Pairs/Data_ConsecLinkPairs_"
//                                        + std::to_string(sysParams.randomSeed)
//                                        + "_" + std::to_string(numRelays)
//                                        + "_" + std::to_string(numBSs) + "_" + std::to_string(selectedGrids.size())
//                                        + "_" +
//                                        std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180)) +
//                                        ".txt";
//      std::ifstream fileConsecutive(dataConsecLinkPairs);
//      std::vector<int> consecLinkPairSet;
//      bool writeConsecutive;
//      if (fileConsecutive.good()) {
//          writeConsecutive = false;
//      } else {
//          writeConsecutive = true;
//      }
//      collectConsecutiveLinkPairs(consecLinkPairSet, phyLinkSet, dataConsecLinkPairs, writeConsecutive);

      /*
       * ==========================
       *   Generate mesh topology
       * ==========================
       */
//  std::vector<std::vector<double>> eHopMap;
//  evaluateEstimateHopNumbers(eHopMap, bsSet, eHops);
      std::vector<std::vector<int>> nodeConnections(numBSs, std::vector<int>());
      std::vector<std::vector<int>> treeConnections;
      std::vector<std::vector<Point_t>> bsPairs;
//  primAlgorithm(eHopMap, bsSet, bsSet.size()/2, nodeConnections, treeConnections, bsPairs);
//  primAlgorithmSetLinksToGateway(eHopMap, bsSet, 19, sysParams.minConnectionsAtMBs, nodeConnections, treeConnections, bsPairs);
      /* File to store the consecutive link pairs info. */
      std::string dataTopology = "../Data/Topology/Data_Topology_" + sysParams.topologyType
                                 + "_" + std::to_string(sysParams.randomSeed)
                                 + "_" + std::to_string(numRelays)
                                 + "_" + std::to_string(numBSs) + ".txt";
      treeTopologyMeshAtlanta(mBSPos, bsGridMap, bsLocation, bsSet, nodeConnections, treeConnections, bsPairs);
      printConnections(nodeConnections);
      std::ifstream fileTopology(dataTopology);
      if (!fileTopology.good()) {
          writeTopologyToFile(dataTopology, treeConnections, numRelays);
      }


      /*
       * ====================================================
       *   Collect first/last hop candidate physical links.
       * ====================================================
       */
//      std::string dataFirstHop = "../Data/First_Hop_Set/Data_FirstHopSet_" + std::to_string(sysParams.randomSeed)
//                                 + "_" + std::to_string(treeConnections.size())
//                                 + "_" + std::to_string(phyLinkSet.size())
//                                 + "_" + std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180)) +
//                                 ".txt";
//      std::string dataLastHop = "../Data/Last_Hop_Set/Data_LastHopSet_" + std::to_string(sysParams.randomSeed)
//                                + "_" + std::to_string(treeConnections.size())
//                                + "_" + std::to_string(phyLinkSet.size())
//                                + "_" + std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180)) +
//                                ".txt";
//      std::vector<std::vector<int>> firstHopSet(treeConnections.size(), std::vector<int>(phyLinkSet.size()));
//      std::vector<std::vector<int>> lastHopSet(treeConnections.size(), std::vector<int>(phyLinkSet.size()));
//      std::ifstream fileFirstHop(dataFirstHop);
//      bool writeFirstLastHop = false;
//      if (!fileFirstHop.good()) writeFirstLastHop = true;
//      collectFirstLastHopCandidatePhyLinks(firstHopSet, lastHopSet, phyLinkSet, treeConnections, numRelays,
//                                           writeFirstLastHop, dataFirstHop, dataLastHop);

      /*
       * ============================================
       *   Collect mutual interference information.
       * ============================================
       */
//      std::string dataMutualInterference = "../Data/Mutual_Interference/Data_MutualInterference_"
//                                           + std::to_string(sysParams.randomSeed)
//                                           + "_" + std::to_string(phyLinkSet.size())
//                                           + "_" +
//                                           std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180)) +
//                                           ".txt";
//      std::ifstream fileMutualInt(dataMutualInterference);
//      bool writeMutualInt = false;
//      if (!fileMutualInt.good()) writeMutualInt = true;
//      std::vector<std::vector<int>> mutualInterferenceIndicator(phyLinkSet.size(), std::vector<int>(phyLinkSet.size()));
//      collectMutualInterferenceInfo(mutualInterferenceIndicator, phyLinkSet, numRelays, allNodes, sysParams,
//                                    dataMutualInterference, writeMutualInt);


//  /*
//   * =======================================================================================
//   * If roof top relays are in use, select roof top relays according to the grid constraint.
//   * =======================================================================================
//   */
//  if (sysParams.relayType.compare("Top") == 0) {
//    selectRelayPerGrid(roofTopRelays, sysParams);
//  }
//  /*
//   * ==================================================================================================================
//   * Check if connectivity information exists, if not, generate connectivity information; otherwise, read existed info.
//   * ==================================================================================================================
//   */
//  std::ifstream fileConnects(dataRelayNeighbors);
//  std::vector<std::vector<int>> relayNeighborList;
//  if (fileConnects.good()){
//    // read the relay neighborList
//    getRelayNeighborInfoFromFile(relayNeighborList, dataRelayNeighbors);
//  } else {
//     exploreConnectivity(relayNeighborList, allRelays, buildingSet, dataRelayNeighbors);
//  }

//  /*
//   * ==============================================================
//   * Add source and destination bss to "nodes" and "neighbor list".
//   * ==============================================================
//   */
//  nodeNeighborList = relayNeighborList;
//  allNodes = allRelays;
//  for (auto bs : bsSet){
//    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, bs, allNodes, buildingSet);
//    allNodes.push_back(bs);
//  }



//  for (int i = allRelays.size(); i < allNodes.size(); i++) {
//    int sBSSD = evaluateSpaceDiversityAtNode(i, allNodes,
//                                            maxSDNodeList, nodeNeighborList, sysParams);
//    cout << "The space diversity at the " << i-allRelays.size() << "-th small cell base station is: " << sBSSD << endl;
//  }
//
//  return 0;

      /*
       * =========================================================================
       * For each logical link in the tree/mesh, run the path selection alogrithm.
       * =========================================================================
       */

      int extraHopNum = 0;
      /*
       * Assume there are at most 9999 nodes in the network, thus, source node id * 10000 + destination node id
       * can index the source and destination pair of a physical link.
       */
      std::map<int, Vector_t> selectedPhysicalLinks;
      std::vector<int> selectedRelays;
      std::map<int, std::vector<Vector_t>> phyLinksAtBSs;
      std::map<int, Vector_t>::iterator pLinkIter;
      std::vector<std::vector<int>> allPaths;
      std::vector<int> noPathList;
      bool feasible = true;
//  std::shuffle(treeConnections.begin(), treeConnections.end(), std::default_random_engine(sysParams.randomSeed));
      std::vector<int> tempConnection;
//  tempConnection = treeConnections[16];
//  treeConnections[16] = treeConnections[8];
//  treeConnections[8] = tempConnection;
//  tempConnection = treeConnections[21];
//  treeConnections[21] = treeConnections[5];
//  treeConnections[5] = tempConnection;
//  tempConnection = treeConnections[35];
//  treeConnections[35] = treeConnections[1];
//  treeConnections[1] = tempConnection;
      std::vector<int> sequence;
      for (int i = 0; i < treeConnections.size(); i++) {
          sequence.push_back(i);
      }
      for (int i = 0; i < treeConnections.size(); i++) {
          if (!feasible) {
              break;
          }
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

              if (j > 4) {
                  cout << "There is no available path." << endl;
                  noPathList.push_back(i);
                  if (i >= 1) {
                      int k = i - 1;
                      while (k >= 0 && sequence[k] > sequence[i]) {
                          k--;
                      }
                      if (k < 0) {
                          cout << "(W) The algorithm cannot find the feasible solution." << endl;
                          feasible = false;
                      } else {
                          tempConnection = treeConnections[i];
                          treeConnections[i] = treeConnections[k];
                          treeConnections[k] = tempConnection;
                          int tempSequence = sequence[i];
                          sequence[i] = sequence[k];
                          sequence[k] = tempSequence;
                          selectedPhysicalLinks.clear();
                          selectedRelays.clear();
                          phyLinksAtBSs.clear();
                          allPaths.clear();
                          noPathList.clear();
                          i = -1;
                      }

                  } else {
                      cout << "(W) The algorithm cannot find the feasible solution." << endl;
                      feasible = false;
                  }

                  break;
              }

              searchPathDecodeForwardMaxHop(pathList, allNodes, nodeNeighborList, numRelays, sysParams, phyLinksAtBSs,
                                            selectedPhysicalLinks, selectedRelays);
              if (pathList.pathList.size() > 0) {
                  caseCount++;
                  std::vector<int> tempPath;
                  if (j == 1) {
                      /* If the set of paths found are with single hop, the single hop path must be recorded. */
                      assert(pathList.getSingleHopMaxThroughputId() >= 0);
                      tempPath = pathList.pathList[pathList.getSingleHopMaxThroughputId()];
                  } else if (pathList.getMultiHopMaxThroughputId() >= 0) {
                      /* If the set of paths are supposed to have more than 1 hops, and the multi-hop paths are available. */
                      tempPath = pathList.pathList[pathList.getMultiHopMaxThroughputId()];
                  } else {
                      /* Though the paths found are supposed to have more than 1 hops, but only single hop path is found. */
                      tempPath = pathList.pathList[pathList.getSingleHopMaxThroughputId()];
                  }
                  /* Add the newly found path into the allPaths vector. */
                  allPaths.push_back(tempPath);
                  recordPhysicalLinksInAPath(selectedPhysicalLinks, tempPath, allNodes, sysParams);
                  recordRelaysInAPath(selectedRelays, tempPath, allNodes, sysParams);
                  collectPhyLinksAtBSs(phyLinksAtBSs, tempPath, allNodes);
              }
              cout << "There are " << pathList.pathList.size() << " paths with no more than " << j
                   << " hops being found.\n";
              cout << "---------------------------------------------------------" << endl;

          }

      }
      if (feasible) {
          cout << "The program finds " << allPaths.size() << " paths among " << treeConnections.size() << " paths."
               << endl;
          for (auto path : noPathList) {
              cout << "There is no path available for the " << path << "-th path from BS" << treeConnections[path][0] <<
                   " to BS" << treeConnections[path][1] << endl;
          }
          cout << selectedPhysicalLinks.size() << endl;
          cout << phyLinksAtBSs.size() << endl;
          assert(allPaths.size() == treeConnections.size() * (extraHopNum + 1));
          /* Count the total number of relays used. */
          int numRelaysNeeded = 0;
          for (int i = 0; i < treeConnections.size(); i++) {
              numRelaysNeeded += (allPaths[i * (extraHopNum + 1)].size() - 2);
          }
          cout << "In total, " << numRelaysNeeded << " relays need to be deployed." << endl;

          std::string dataPaths = "../Data/Paths/Data_Results_" + std::to_string(sysParams.randomSeed)
                                  + "_" + std::to_string(numRelays)
                                  + "_" + std::to_string(numBSs)
                                  + "_" + std::to_string((int) round(sysParams.antennaBeamWidth_phi / M_PI * 180));

          if (allPaths.size() == treeConnections.size()) {
              writePathsToFile(allPaths, sequence, allNodes, sysParams, dataPaths);
          }


          std::map<int, std::vector<int>> nodeCheckMinHop;
          std::map<int, std::vector<int>>::iterator it;
          for (int i = 0; i < treeConnections.size(); i++) {
              for (int j = 1; j < allPaths[i * (extraHopNum + 1)].size() - 1; ++j) {
                  it = nodeCheckMinHop.find(allPaths[i * (extraHopNum + 1)][j]);
                  if (it != nodeCheckMinHop.end()) {
                      nodeCheckMinHop.at(allPaths[i * (extraHopNum + 1)][j]).push_back(i);
                  } else {
                      std::vector<int> newPair;
                      newPair.push_back(i);
                      nodeCheckMinHop.insert(
                              std::pair<int, std::vector<int>>(allPaths[i * (extraHopNum + 1)][j], newPair));
                  }
              }
          }

          cout << "========== Check the min hop case ===========" << endl;
          for (it = nodeCheckMinHop.begin(); it != nodeCheckMinHop.end(); ++it) {
              if (it->second.size() > 1) {
                  cout << "-----------------------------------------------------------" << endl;
                  cout << "Relay No. " << it->first << " has been selected by " << it->second.size() << " paths."
                       << endl;
                  int numPath = it->second.size();
                  std::vector<int> pathIDs = it->second;
                  for (int j = 0; j < numPath - 1; j++) {
                      std::vector<int> path1 = allPaths[pathIDs[j] * (extraHopNum + 1)];
                      Point_t src1 = bsPairs[pathIDs[j]][0];
                      Point_t dst1 = bsPairs[pathIDs[j]][1];
                      for (int k = j + 1; k < numPath; k++) {
                          std::vector<int> path2 = allPaths[pathIDs[k] * (extraHopNum + 1)];
                          Point_t src2 = bsPairs[pathIDs[k]][0];
                          Point_t dst2 = bsPairs[pathIDs[k]][1];
                          std::vector<double> distance;
                          distance.push_back(src1.distanceTo(src2));
                          distance.push_back(src1.distanceTo(dst2));
                          distance.push_back(dst1.distanceTo(src2));
                          distance.push_back(dst1.distanceTo(dst2));
                          double minDistance = GetMin(distance);
                          cout << "Path " << pathIDs[j] << ": src - " << src1.toString() << ", dst - "
                               << dst1.toString()
                               << "\n"
                               << "Path " << pathIDs[k] << ": src - " << src2.toString() << ", dst - "
                               << dst2.toString()
                               << "\n"
                               << "Minimum distance between end points: " << minDistance << endl;
//          cout << minDistance << src1.distanceTo(src2) << "\t" << src1.distanceTo(dst2) << "\t"
//               << dst1.distanceTo(src2) << "\t" << dst1.distanceTo(dst2) << endl;
                      }
                  }
              }
          }

          // Check interference cases:
          int countIntPairs = 0;
          for (int i = 0; i < treeConnections.size() - 1; ++i) {
              std::vector<int> path1 = allPaths[i * (extraHopNum + 1)];
              std::vector<Point_t> sd1 = bsPairs[i];
              for (int j = i + 1; j < treeConnections.size(); ++j) {
                  std::vector<int> path2 = allPaths[j * (extraHopNum + 1)];
                  std::vector<Point_t> sd2 = bsPairs[j];
                  bool intTest = checkTwoPathsInterference(path1, path2, sd1, sd2, nodeNeighborList, buildingSet,
                                                           allNodes,
                                                           sysParams);
                  if (intTest) {
                      cout << "Path " << i << " and Path " << j << " interfere with each other." << endl;
                      countIntPairs++;
                  }
              }
          }
          cout << "In total, there are " << countIntPairs << " pairs of paths interfere with each other." << endl;
      }
  }
  cout << "==================================" << endl;
  cout << "This is the end of the simulation." << endl;
  cout << "==================================" << endl;






  return 0;
}