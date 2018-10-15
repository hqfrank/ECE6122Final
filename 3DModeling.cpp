//
// Created by qiang on 4/16/18.
//

#include "3DModeling.h"

/*
 * ============================
 *   Write path data to file.
 * ============================
 */
 void writePathsToFile(const std::vector<std::vector<int>>& allPaths, const std::vector<int>& sequence,
                       const std::vector<Point_t>& allNodes, SystemParameters& parameters,
                       std::string& dataPath) {
   std::string gridSize = std::to_string((int) floor(parameters.gridSize_m + 0.5));
   if (!parameters.interPathIntControl) {
     dataPath = "../Data/Paths/Idp/" + gridSize + "/Data_Results_" + dataPath;
   } else if (parameters.splitMacroBS && !parameters.limitMacroCell) {
     dataPath = "../Data/Paths/Double_MBS/" + gridSize + "/Data_Results_" + dataPath;
   } else if (parameters.splitMacroBS && parameters.limitMacroCell) {
       dataPath = "../Data/Paths/Double_MBS_LimitedArea/" + gridSize + "/Data_Results_" + dataPath;
   } else if (!parameters.splitMacroBS && parameters.limitMacroCell) {
       dataPath = "../Data/Paths/Single_MBS_LimitedArea/" + gridSize + "/Data_Results_" + dataPath;
   } else {
     dataPath = "../Data/Paths/Single_MBS/" + gridSize + "/Data_Results_" + dataPath;
   }
   std::string filePath = dataPath + "_Paths.txt";
   std::string fileCapacity = dataPath + "_Capacity.txt";
   std::string fileSequence = dataPath + "_Sequence.txt";

   std::ofstream outPath, outCapacity, outSequence;
   outPath.open(filePath, std::ios_base::trunc);
   outCapacity.open(fileCapacity, std::ios_base::trunc);
   outSequence.open(fileSequence, std::ios_base::trunc);

   if (outPath.is_open() && outCapacity.is_open() && outSequence.is_open()) {
     int totalRelays = 0;
     for (int i = 0; i < allPaths.size(); ++i) {
       auto cur_seq = sequence[i];
       auto cur_relay = allPaths[i].size() - 2;
       totalRelays = totalRelays + cur_relay;
       outSequence << std::to_string(cur_seq) << "\t" << std::to_string(cur_relay) << endl;
       for (auto n : allPaths[i]) {
         outPath << std::to_string(n) << "\t";
       }
       outPath << endl;
       double pre_cap = 0;
       double path_cap = 10000;
       for (int j = 0; j < allPaths[i].size() - 1; ++j) {
         Point_t p0 = allNodes[allPaths[i][j]];
         Point_t p1 = allNodes[allPaths[i][j+1]];
         double dist = p0.distanceTo(p1);
         double cur_cap = calculateLinkCapacity_Gbps(dist, parameters);
         if (allPaths[i].size() == 2) {
           path_cap = cur_cap;
           outCapacity << std::to_string(path_cap) << endl;
         } else if (j > 0) {
           double cur_cc = pre_cap * cur_cap / (pre_cap + cur_cap);
           if (cur_cc < path_cap) {
             path_cap = cur_cc;
           }
         }
         pre_cap = cur_cap;
       }
       if (allPaths[i].size() > 2) {
         outCapacity << std::to_string(path_cap) << endl;
       }
     }
     outSequence << std::to_string(-1) << "\t" << std::to_string(totalRelays) << endl;
   } else {
     cout << "(E) Some files cannot open." << endl;
   }
   outCapacity.close();
   outPath.close();
   outSequence.close();
 }


/*
 * =========================================================
 * Print out the connections on each node in a tree or mesh.
 * =========================================================
 */
void printConnections(const std::vector<std::vector<int>>& nodeConnections){
  cout << "****************************************************" << endl;
  cout << "Print out the neighbors of each node in the network." << endl;
  cout << "****************************************************" << endl;
  int count = 0;
  for (auto nodeConnection : nodeConnections){
    cout << "BS " << count << " connects to " << nodeConnection.size() << " BSs: ";
    for (auto node : nodeConnection) {
      cout << node << ", ";
    }
    cout << endl;
    count++;
  }
  cout << "----------------------------------------------------" << endl;
}

/*
 * =================================================
 * Prim algorithm to find the minimum spanning tree.
 * =================================================
 */
void primAlgorithm(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
                   const int mBSId, std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                   std::vector<std::vector<Point_t>>& bsPairs){
  double key[bsSet.size()];
  int otherEnd[bsSet.size()];
  for (int i = 0; i < bsSet.size(); i++) {
    key[i] = 100000;
    otherEnd[i] = -1;
  }
  key[mBSId] = 0;
  otherEnd[mBSId] = mBSId;
  std::vector<int> unselectedBSs;
  for (int i = 0; i < bsSet.size(); i++){
    unselectedBSs.push_back(i);
  }
  std::vector<int> selectedBSs;
  while(selectedBSs.size() < bsSet.size()){
    // Find the node with the min key value in the unselected set.
    double minKey = 100000;
    int minKeyBSId = -1;
    int posOfMinKeyBS = -1;
    for (int i = 0; i < unselectedBSs.size(); i++) {
      if (key[unselectedBSs.at(i)] < minKey) {
        minKey = key[unselectedBSs.at(i)];
        minKeyBSId = unselectedBSs.at(i);
        posOfMinKeyBS = i;
      }
    }
    if (minKeyBSId == -1) {
      cerr << "Error! There is no valid key value in the unselected set." << endl;
      exit(errno);
    }
    if (minKey > 0) {
      connections.at(minKeyBSId).push_back(otherEnd[minKeyBSId]);
      connections.at(otherEnd[minKeyBSId]).push_back(minKeyBSId);
      cout << "BS " << minKeyBSId << "\t---\t" << "BS " << otherEnd[minKeyBSId] << endl;
      // Add this connection to the tree
      std::vector<int> curConnection;
      curConnection.push_back(minKeyBSId);
      curConnection.push_back(otherEnd[minKeyBSId]);
      tree.push_back(curConnection);
      // Add this connection to bs pairs
      std::vector<Point_t> curBSPair;
      curBSPair.push_back(bsSet.at(minKeyBSId));
      curBSPair.push_back(bsSet.at(otherEnd[minKeyBSId]));
      bsPairs.push_back(curBSPair);

    }

    // move the node with min key value from unselected set to selected set.
    selectedBSs.push_back(minKeyBSId);
    unselectedBSs.erase(unselectedBSs.begin()+posOfMinKeyBS);


    std::vector<double> neighborDist = eHopMaps.at(minKeyBSId);
    for (int i = 0; i < neighborDist.size(); i++) {
      if (key[i] > neighborDist.at(i)) {
        key[i] = neighborDist.at(i);
        otherEnd[i] = minKeyBSId;
      }
    }
  }
}


/*
 * ================================================================================================================
 * Modified Prim algorithm to find the minimum spanning tree with a number of links connecting to the gateway node.
 * ================================================================================================================
 */
void primAlgorithmSetLinksToGateway(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
                                    const int mBSId, const int numLinksToGateway,
                                    std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                                    std::vector<std::vector<Point_t>>& bsPairs){
    /* The number of links to gateway node (marco-base station) should be smaller than the total number of links. */
    assert(numLinksToGateway < bsSet.size());
    /* Initialization. */
    double key[bsSet.size()];
    int otherEnd[bsSet.size()];
    for (int i = 0; i < bsSet.size(); i++) {
        key[i] = 100000;
        otherEnd[i] = -1;
    }
    key[mBSId] = 0;
    otherEnd[mBSId] = mBSId;
    std::vector<int> unselectedBSs;
    for (int i = 0; i < bsSet.size(); i++){
        unselectedBSs.push_back(i);
    }
    std::vector<int> selectedBSs;
    /* Select numLinksToGateway base stations connecting to macro-base station first. */
    int countLinksToGateway = -1;
    std::vector<double> mBSNeighborDist = eHopMaps[mBSId];
    std::vector<int> mBSNeighborDistSortedIndex = sort_indexes(mBSNeighborDist);
    while(selectedBSs.size() < bsSet.size()){
        // Find the node with the min key value in the unselected set.
        double minKey = 100000;
        int minKeyBSId = -1;
        int posOfMinKeyBS = -1;
        for (int i = 0; i < unselectedBSs.size(); i++) {
            if (countLinksToGateway < numLinksToGateway && countLinksToGateway > -1) {
                if (unselectedBSs[i] == mBSNeighborDistSortedIndex[countLinksToGateway+1]) {
                    minKey = key[unselectedBSs.at(i)];
                    minKeyBSId = unselectedBSs.at(i);
                    posOfMinKeyBS = i;
                }
            } else {
                if (key[unselectedBSs.at(i)] < minKey) {
                    minKey = key[unselectedBSs.at(i)];
                    minKeyBSId = unselectedBSs.at(i);
                    posOfMinKeyBS = i;
                }
            }
        }
        if (minKeyBSId == -1) {
            cerr << "Error! There is no valid key value in the unselected set." << endl;
            exit(errno);
        }
        if (minKey > 0) {
            connections.at(minKeyBSId).push_back(otherEnd[minKeyBSId]);
            connections.at(otherEnd[minKeyBSId]).push_back(minKeyBSId);
            cout << "BS " << minKeyBSId << "\t---\t" << "BS " << otherEnd[minKeyBSId] << endl;
            // Add this connection to the tree
            std::vector<int> curConnection;
            curConnection.push_back(minKeyBSId);
            curConnection.push_back(otherEnd[minKeyBSId]);
            tree.push_back(curConnection);
            // Add this connection to bs pairs
            std::vector<Point_t> curBSPair;
            curBSPair.push_back(bsSet.at(minKeyBSId));
            curBSPair.push_back(bsSet.at(otherEnd[minKeyBSId]));
            bsPairs.push_back(curBSPair);

        }

        // move the node with min key value from unselected set to selected set.
        selectedBSs.push_back(minKeyBSId);
        unselectedBSs.erase(unselectedBSs.begin()+posOfMinKeyBS);


        std::vector<double> neighborDist = eHopMaps.at(minKeyBSId);

        if (countLinksToGateway < numLinksToGateway) {
            for (int i = 0; i < neighborDist.size(); i++) {
                if (key[i] > neighborDist.at(i) && otherEnd[i] != mBSId) {
                    key[i] = neighborDist.at(i);
                    otherEnd[i] = minKeyBSId;
                }
            }
        } else {
            for (int i = 0; i < neighborDist.size(); i++) {
                if (key[i] > neighborDist.at(i)) {
                    key[i] = neighborDist.at(i);
                    otherEnd[i] = minKeyBSId;
                }
            }
        }


        countLinksToGateway++;
    }
}

/*
 * ================================================================================================================
 * Modified Prim algorithm to find the minimum spanning tree with a number of links connecting to the gateway node.
 * ================================================================================================================
 */
//void primAlgorithmEightLinksAtMBS(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
//                                    const int mBSId, const SystemParameters& parameters,
//                                    std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
//                                    std::vector<std::vector<Point_t>>& bsPairs){
//    /* Get the number of grids along x and y axis. */
//    auto numGridAlongX = (int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
//    auto numGridAlongY = (int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
//    /* The macro base station should have 8 neighbors around. */
//    assert((mBSId > numGridAlongX) && (mBSId < numGridAlongX * (numGridAlongY - 1) - 1));
//    int eightNeighborGrids[8] = {mBSId - numGridAlongX - 1, mBSId - numGridAlongX, mBSId - numGridAlongX + 1, mBSId - 1,
//                                 mBSId + 1, mBSId + numGridAlongX - 1, mBSId + numGridAlongX, mBSId + numGridAlongX + 1};
//
//    /* Initialization. */
//    double key[bsSet.size()];
//    int otherEnd[bsSet.size()];
//    for (int i = 0; i < bsSet.size(); i++) {
//        key[i] = 100000;
//        otherEnd[i] = -1;
//    }
//    key[mBSId] = 0;
//    otherEnd[mBSId] = mBSId;
//    std::vector<int> unselectedBSs;
//    for (int i = 0; i < bsSet.size(); i++){
//        unselectedBSs.push_back(i);
//    }
//    std::vector<int> selectedBSs;
//    /* Select numLinksToGateway base stations connecting to macro-base station first. */
//    int countLinksToGateway = -1;
//    std::vector<double> mBSNeighborDist = eHopMaps[mBSId];
//    std::vector<int> mBSNeighborDistSortedIndex = sort_indexes(mBSNeighborDist);
//    while(selectedBSs.size() < bsSet.size()){
//        // Find the node with the min key value in the unselected set.
//        double minKey = 100000;
//        int minKeyBSId = -1;
//        int posOfMinKeyBS = -1;
//        for (int i = 0; i < unselectedBSs.size(); i++) {
//            if (countLinksToGateway < numLinksToGateway && countLinksToGateway > -1) {
//                if (unselectedBSs[i] == mBSNeighborDistSortedIndex[countLinksToGateway+1]) {
//                    minKey = key[unselectedBSs.at(i)];
//                    minKeyBSId = unselectedBSs.at(i);
//                    posOfMinKeyBS = i;
//                }
//            } else {
//                if (key[unselectedBSs.at(i)] < minKey) {
//                    minKey = key[unselectedBSs.at(i)];
//                    minKeyBSId = unselectedBSs.at(i);
//                    posOfMinKeyBS = i;
//                }
//            }
//        }
//        if (minKeyBSId == -1) {
//            cerr << "Error! There is no valid key value in the unselected set." << endl;
//            exit(errno);
//        }
//        if (minKey > 0) {
//            connections.at(minKeyBSId).push_back(otherEnd[minKeyBSId]);
//            connections.at(otherEnd[minKeyBSId]).push_back(minKeyBSId);
//            cout << "BS " << minKeyBSId << "\t---\t" << "BS " << otherEnd[minKeyBSId] << endl;
//            // Add this connection to the tree
//            std::vector<int> curConnection;
//            curConnection.push_back(minKeyBSId);
//            curConnection.push_back(otherEnd[minKeyBSId]);
//            tree.push_back(curConnection);
//            // Add this connection to bs pairs
//            std::vector<Point_t> curBSPair;
//            curBSPair.push_back(bsSet.at(minKeyBSId));
//            curBSPair.push_back(bsSet.at(otherEnd[minKeyBSId]));
//            bsPairs.push_back(curBSPair);
//
//        }
//
//        // move the node with min key value from unselected set to selected set.
//        selectedBSs.push_back(minKeyBSId);
//        unselectedBSs.erase(unselectedBSs.begin()+posOfMinKeyBS);
//
//
//        std::vector<double> neighborDist = eHopMaps.at(minKeyBSId);
//
//        if (countLinksToGateway < numLinksToGateway) {
//            for (int i = 0; i < neighborDist.size(); i++) {
//                if (key[i] > neighborDist.at(i) && otherEnd[i] != mBSId) {
//                    key[i] = neighborDist.at(i);
//                    otherEnd[i] = minKeyBSId;
//                }
//            }
//        } else {
//            for (int i = 0; i < neighborDist.size(); i++) {
//                if (key[i] > neighborDist.at(i)) {
//                    key[i] = neighborDist.at(i);
//                    otherEnd[i] = minKeyBSId;
//                }
//            }
//        }
//
//
//        countLinksToGateway++;
//    }
//}

///*
// * ================================================================================================================
// * Modified Prim algorithm to find the minimum spanning tree with limited degrees on all nodes except the gateway.
// * ================================================================================================================
// */
//void primAlgorithmLimitedDegree(const std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet,
//                                const int mBSId, const int numLinksToGateway, const int degreeLimit,
//                                std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
//                                std::vector<std::vector<Point_t>>& bsPairs){
//    /* The number of links to gateway node (marco-base station) should be smaller than the total number of links. */
//    assert(numLinksToGateway < bsSet.size());
//    /* Initialization. */
//    double key[bsSet.size()];
//    int otherEnd[bsSet.size()];
//    for (int i = 0; i < bsSet.size(); i++) {
//        key[i] = 100000;
//        otherEnd[i] = -1;
//    }
//    key[mBSId] = 0;
//    otherEnd[mBSId] = mBSId;
//    std::vector<int> unselectedBSs;
//    for (int i = 0; i < bsSet.size(); i++){
//        unselectedBSs.push_back(i);
//    }
//    std::vector<int> selectedBSs;
//    /* Select numLinksToGateway base stations connecting to macro-base station first. */
//    int countLinksToGateway = -1;
//    std::vector<double> mBSNeighborDist = eHopMaps[mBSId];
//    std::vector<int> mBSNeighborDistSortedIndex = sort_indexes(mBSNeighborDist);
//    while(selectedBSs.size() < bsSet.size()){
//        // Find the node with the min key value in the unselected set.
//        double minKey = 100000;
//        int minKeyBSId = -1;
//        int posOfMinKeyBS = -1;
//        for (int i = 0; i < unselectedBSs.size(); i++) {
//            if (countLinksToGateway < numLinksToGateway && countLinksToGateway > -1) {
//                if (unselectedBSs[i] == mBSNeighborDistSortedIndex[countLinksToGateway+1]) {
//                    minKey = key[unselectedBSs.at(i)];
//                    minKeyBSId = unselectedBSs.at(i);
//                    posOfMinKeyBS = i;
//                }
//            } else {
//                if (key[unselectedBSs.at(i)] < minKey) {
//                    minKey = key[unselectedBSs.at(i)];
//                    minKeyBSId = unselectedBSs.at(i);
//                    posOfMinKeyBS = i;
//                }
//            }
//        }
//        if (minKeyBSId == -1) {
//            cerr << "Error! There is no valid key value in the unselected set." << endl;
//            exit(errno);
//        }
//        if (minKey > 0) {
//            connections.at(minKeyBSId).push_back(otherEnd[minKeyBSId]);
//            connections.at(otherEnd[minKeyBSId]).push_back(minKeyBSId);
//            cout << "BS " << minKeyBSId << "\t---\t" << "BS " << otherEnd[minKeyBSId] << endl;
//            // Add this connection to the tree
//            std::vector<int> curConnection;
//            curConnection.push_back(minKeyBSId);
//            curConnection.push_back(otherEnd[minKeyBSId]);
//            tree.push_back(curConnection);
//            // Add this connection to bs pairs
//            std::vector<Point_t> curBSPair;
//            curBSPair.push_back(bsSet.at(minKeyBSId));
//            curBSPair.push_back(bsSet.at(otherEnd[minKeyBSId]));
//            bsPairs.push_back(curBSPair);
//
//        }
//
//        // move the node with min key value from unselected set to selected set.
//        selectedBSs.push_back(minKeyBSId);
//        unselectedBSs.erase(unselectedBSs.begin()+posOfMinKeyBS);
//
//
//        std::vector<double> neighborDist = eHopMaps.at(minKeyBSId);
//
//        if (countLinksToGateway < numLinksToGateway) {
//            for (int i = 0; i < neighborDist.size(); i++) {
//                if (key[i] > neighborDist.at(i) && otherEnd[i] != mBSId) {
//                    key[i] = neighborDist.at(i);
//                    otherEnd[i] = minKeyBSId;
//                }
//            }
//        } else {
//            for (int i = 0; i < neighborDist.size(); i++) {
//                if (key[i] > neighborDist.at(i)) {
//                    key[i] = neighborDist.at(i);
//                    otherEnd[i] = minKeyBSId;
//                }
//            }
//        }
//
//
//        countLinksToGateway++;
//    }
//}

/*
 * ==============================================================================================
 * Approximately evaluate the estimated hops needed between each pair of source and destinations.
 * ==============================================================================================
 */
void evaluateEstimateHopNumbers(std::vector<std::vector<double>>& eHopMaps, const std::vector<Point_t>& bsSet, const EstimatedHop& eHops){
  auto numBSs = bsSet.size();
  for (int i = 0; i < numBSs; i++){
    Point_t pi = bsSet.at(i);
    std::vector<double> eHopPi;
    for (int j = 0; j < numBSs; j++) {
      Point_t pj = bsSet.at(j);
      double dist = pi.distanceTo(pj);
      if (dist < 0.1) {
        eHopPi.push_back(0);
      } else if (dist <= 200) {
        eHopPi.push_back(eHops.l200g20);
      } else if (dist <= 400) {
        eHopPi.push_back(eHops.l400g200);
      } else if (dist <= 600) {
        eHopPi.push_back(eHops.l600g400);
      } else if (dist <= 800) {
        eHopPi.push_back(eHops.l800g600);
      } else if (dist <= 1000) {
        eHopPi.push_back(eHops.l1000g800);
      } else {
        eHopPi.push_back(10000);
      }
    }
    eHopMaps.push_back(eHopPi);
  }

}


/*
 * Calculate the point at the position p between p1 and p2, such that |p1p|= pro*|p1p2|
 */
Point_t proportionalPoint(const Point_t& p1, const Point_t& p2, double pro){
  assert(pro >= 0);
  Vector_t v12(p1, p2);
  Vector_t v(v12, pro);
  Point_t p = p1.destinationTo(v);
  return p;
}

/*
 * Dot product of two vectors
 */
double dot(const Vector_t& v1, const Vector_t& v2){
  return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY()) + (v1.getZ() * v2.getZ());
}

Vector_t cross(const Vector_t& v1, const Vector_t& v2){
  Vector_t v(v1.getY() * v2.getZ() - v1.getZ() * v2.getY(), v1.getZ() * v2.getX() - v1.getX() * v2.getZ(), v1.getX() * v2.getY() - v1.getY() * v2.getX());

  return v;
}

Vector_t normalize(const Vector_t& v) {
  return v.divide(v.mod());
}

/*
 * =================================================
 *   Read in building information from data files.
 * =================================================
 */
 void getBuildingInfoFromFile(std::vector<Building_t>& buildings, std::string dataBuildings,
                              std::string dataBuildingVertices, SystemParameters& parameters) {
  std::ifstream fileIn(dataBuildings);
  std::ofstream fileOut;
  std::string str;
  std::vector<std::string> data;
  while (std::getline(fileIn, str, '\t')) {
    if (str.find('\n') != std::string::npos) {
      data.push_back(str.substr(0, str.find('\n')));
      data.push_back(str.substr(str.find('\n')+1));
    } else {
      data.push_back(str);
    }
  }

  // Create each building objects and store them.
  fileOut.open(dataBuildingVertices, std::ios_base::trunc);
  if (fileOut.is_open()) {
    cout << "(O) Ready to write building vertices information to file." << endl;
  } else {
    cout << "(E) Fail to open the file in function getBuildingInfoFromFile()." << endl;
  }
  for (int i = 0; i < data.size(); i = i+7) {
    double center[2] {std::stod(data[i]), std::stod(data[i+1])};  //  the center of a building i
    double length_m = std::stod(data[i+2]);  // the length of the rectangle (i.e., building)
    double width_m = std::stod(data[i+3]);   // the width of the rectangle (i.e., building)
    double topHeight_m = std::stod(data[i+4]);  // the height of the top ceiling of a building
    double baseLevel_m = std::stod(data[i+5]);  // the height of the base of a building
    double lwhbg[] {length_m, width_m, topHeight_m, baseLevel_m + parameters.minHeightForRelay_m,
                    parameters.groundLevel_m};  // calculate the min height to deploy relays on the building
    double orientation_rad = (std::stod(data[i+6]))/180.0*M_PI;
    /* Generate each building object. */
    Building_t newBuilding(center, lwhbg, orientation_rad, parameters.maxHeightForRelay_m,
                           parameters.densityRelayOnBuilding, i/7*parameters.randomSeed,
                           parameters.minNumRelaysPerFace);  // do not deploy relay on top face above 200 m
    buildings.push_back(newBuilding);
    fileOut << (buildings[buildings.size()-1].toStringData()+"\n");
  }
  fileOut.close();
}

std::vector<Point_t> generateCandidateBaseStations(std::vector<Building_t>& buildingSet, std::vector<Point_t>& roofTopRelays, SystemParameters& parameters){
  /* Set up random generator. */
  // Seed with a real random value, if available
  std::random_device r;
//  std::default_random_engine e(r());
  std::default_random_engine e(parameters.randomSeed);
  std::uniform_int_distribution<int> uniform_dist(0, 999);

  /* Initialize variables for storing BSs' locations. */
  std::vector<Point_t> poolBS;
  int numBuildings = buildingSet.size();
  int countBS = 0;
  /* Pick one of the vertices of a building satisfying the height requirement as a candidate BS. */
  for(unsigned int i=0; i<numBuildings; i++){
    /* Test the building's height. */
    double minHeightBuilding = buildingSet.at(i).getHeightBase() - parameters.minHeightForRelay_m + parameters.minHeightForBS_m;
    double maxHeightBuilding = buildingSet.at(i).getHeightBase() - parameters.minHeightForRelay_m + parameters.maxHeightForBS_m;
    double randomDouble = uniform_dist(e) / 1000.0;
    auto caseId = (unsigned int) floor(randomDouble * 4.0);
    if (buildingSet.at(i).getHeight() >= minHeightBuilding && buildingSet.at(i).getHeight() <= maxHeightBuilding) {
      // Read all 4 top vertices of building i
      std::vector<Point_t> topVertices = buildingSet.at(i).getVts();
      poolBS.push_back(topVertices.at((caseId++) % 4));
      // This building has a candidate BS location.
      buildingSet.at(i).setHasBS(true);
      // Add the other 3 top corners as the roof top relay locations.
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
    }
    if (buildingSet.at(i).getHeight() > maxHeightBuilding){
      // Calculate the effective building height.
      double buildingHeight = buildingSet.at(i).getHeight() - buildingSet.at(i).getHeightBase();
      double randomHeight = uniform_dist(e) / 1000.0 * (parameters.maxHeightForBS_m-parameters.minHeightForBS_m) + parameters.minHeightForBS_m;
      double pro = randomHeight / buildingHeight;
      // Read all 4 top vertices of building i
      std::vector<Point_t> topVertices = buildingSet.at(i).getVts();
      std::vector<Point_t> baseVertices = buildingSet.at(i).getVbs();
      Point_t newBS(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId) % 4), pro));
      poolBS.push_back(newBS);
      caseId++;
      Point_t newRelay0(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId) % 4), pro));
      caseId++;
      roofTopRelays.push_back(newRelay0);
      Point_t newRelay1(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId) % 4), pro));
      caseId++;
      roofTopRelays.push_back(newRelay1);
      Point_t newRelay2(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId) % 4), pro));
      roofTopRelays.push_back(newRelay2);
      // System.out.println("No. "+ countBS+ "\t BS candidate position at " + candiBS);
      // StdDraw.point(candiBS.x, candiBS.y);
    }
  }
  cout << "(*) There are " + to_string(poolBS.size()) + " candidate base stations being generated." << endl;
  cout << "(*) There are " + to_string(roofTopRelays.size()) + " candidate roof top relays being generated." << endl;

  return poolBS;
}

void selectBaseStationPerGrid(std::vector<Point_t>& bsSet, std::vector<std::vector<int>>& bsGridMap,
                              std::vector<std::vector<int>>& bsLocation, std::vector<std::vector<int>>& numRelaysInGrid,
                              std::string& dataBSs, bool write, SystemParameters& parameters){
  /* Number of grids. */
  auto numGridAlongX = (unsigned int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
  auto numGridAlongY = (unsigned int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
  /* Initialize the Map for indicating whether there is a BS in the grid. */
  std::vector<bool> gridHasBS;
//  std::vector<Point_t> bsInGridLocal(numGridAlongX * numGridAlongY);
//  std::vector<std::vector<int>> bsGridMapLocal(numGridAlongX, std::vector<int>(numGridAlongY));
  for (int i = 0; i < numGridAlongX; i++){
    std::vector<int> tempVector;
    for (int j = 0; j < numGridAlongY; j++){
      gridHasBS.push_back(false);
      tempVector.push_back(-1);
    }
    bsGridMap.push_back(tempVector);
  }
  /* Iterating each candidate BS. */
  unsigned int i = 0;
  while(i < bsSet.size()){
    Point_t currentBS = (bsSet.at(i));
    auto gridIndexX = (unsigned int) floor((currentBS.getX() - parameters.areaXRange_m[0])/parameters.gridSize_m);
    auto gridIndexY = (unsigned int) floor((currentBS.getY() - parameters.areaYRange_m[0])/parameters.gridSize_m);
    if (gridHasBS.at(gridIndexX*numGridAlongY + gridIndexY)){
      bsSet.erase(bsSet.begin() + i);
    } else {
      if (parameters.bsFewRelaysControl && numRelaysInGrid[gridIndexX][gridIndexY] < parameters.minRelayNumInGrid) {
        // If too few relays are in this grid, the bs will be removed.
        bsSet.erase(bsSet.begin() + i);
      } else {
        gridHasBS.at(gridIndexX*numGridAlongY + gridIndexY) = true;
        bsGridMap[gridIndexX][gridIndexY] = i;
        i++;
      }
    }
  }

  cout << "(*) There are " + to_string(bsSet.size()) + " candidate base stations being selected." << endl;
  /* Write the selected base stations to file. */
  if (write) {
    ofstream outFile;
    outFile.open(dataBSs, std::ios_base::app);
    if (!outFile.is_open()) {
      cerr << "Error!!!The file to store base stations is not open!!" << endl;
      exit(errno);
    }
    for(auto bs : bsSet) {
      outFile << bs.toStringData() << endl;
    }
    outFile.close();
  }
  /* Print the topology of the base stations. */
  std::vector<std::vector<int>> bsLocationTemp(bsSet.size(), std::vector<int>(2));
  cout << "===================== The topology of base stations ====================" << endl;
  for (int j = 0; j < numGridAlongX; j++){
    for (int k = 0; k < numGridAlongY; k++) {
      if (gridHasBS[j * numGridAlongY + k] == true) {
        bsLocationTemp[bsGridMap[j][k]][0] = j;
        bsLocationTemp[bsGridMap[j][k]][1] = k;
        if (bsGridMap[j][k] < 10) {
          cout << "BS0" << bsGridMap[j][k] << "\t";
        } else {
          cout << "BS" << bsGridMap[j][k] << "\t";
        }
      } else {
        cout << "NULL\t";
      }
    }
    cout << endl;
  }
  cout << "------------------------------------------------------------------------" << endl;
  bsLocation = bsLocationTemp;
}

void writeBSsLoactionToFile(const std::vector<std::vector<int>>& bsLocations, const std::string& dataBSsGrid){
    std::ofstream fileBSsGrid;
    fileBSsGrid.open(dataBSsGrid, std::ios_base::app);
    if (!fileBSsGrid.is_open()) {
        cerr << "Error!!!The file to store the grid indices of base stations is not open!!" << endl;
        exit(errno);
    }
    for(auto bsGrid : bsLocations) {
        fileBSsGrid << bsGrid[0] << "\t" << bsGrid[1] << endl;
    }
    fileBSsGrid.close();
}

void writeSpaceDiversityToFile(int randomSeed, int numRelays, int spaceDiversity, const std::string& dataSpaceDiversity){
    std::ofstream fileSpaceDiversity;
    fileSpaceDiversity.open(dataSpaceDiversity, std::ios_base::app);
    if (!fileSpaceDiversity.is_open()) {
        cerr << "Error!!!The file to store the space diversity of base stations is not open!!" << endl;
        exit(errno);
    }
    fileSpaceDiversity << randomSeed << "\t" << numRelays << "\t" << spaceDiversity << endl;
    fileSpaceDiversity.close();
}

void treeTopologyMeshAtlanta(const int mBSPos[2], std::vector<std::vector<int>>& bsGridMap,
                             const std::vector<std::vector<int>>& bsLocation,
                             const std::vector<Point_t>& bsSet,
                             std::vector<std::vector<int>>& connections, std::vector<std::vector<int>>& tree,
                             std::vector<std::vector<Point_t>>& bsPairs, const SystemParameters& parameters){
    /* The grid where the macro cell base station locates is indicated by mBSPos[2] */
    assert(bsGridMap[mBSPos[0]][mBSPos[1]] >= 0);  // The mBS should be valid.
    /* In case, the area is limited. */
    if (parameters.limitMacroCell) {
        int numSmallCells = (int) floor(parameters.macroCellSize_m/parameters.gridSize_m + 0.5);
        int numExtraCells = (numSmallCells - 1)/2;
        for (int i = 0; i < (int) bsGridMap.size(); i++) {
            for (int j = 0; j < (int) bsGridMap[0].size(); j++) {
                if (i < mBSPos[0] - numExtraCells || i > mBSPos[0] + numExtraCells ||
                    j < mBSPos[1] - numExtraCells || j > mBSPos[1] + numExtraCells) {
                    bsGridMap[i][j] = -1;
                }
            }
        }
    }
    /* Initialization. */
    std::vector<int> selectedBS;
    selectedBS.push_back(bsGridMap[mBSPos[0]][mBSPos[1]]);  // First to select the mBS
    int nextRootIndex = 0;
    int nextRoot = selectedBS[nextRootIndex];
    nextRootIndex++;
    if (parameters.splitMacroBS) {
        selectedBS.push_back((int) bsSet.size() - 1);
        nextRootIndex++;
    }
    /* Connect the mBS to its 8 neighbors. */
  for (int i = mBSPos[0] - 1; i <= mBSPos[0] + 1; ++i) {
    for (int j = mBSPos[1] - 1; j <= mBSPos[1] + 1; ++j) {
      if ((i != mBSPos[0] || j != mBSPos[1]) && bsGridMap[i][j] > -1) {
        std::vector<int> curConnection;
        int mBSId = bsGridMap[mBSPos[0]][mBSPos[1]];
        if (parameters.splitMacroBS && !(i == mBSPos[0] || j == mBSPos[1])) {
          // use the last BS in the bsSet as the macro-cell base station
          mBSId = (int) bsSet.size() - 1;
        }
        curConnection.push_back(mBSId);  // add the mBS
        curConnection.push_back(bsGridMap[i][j]);  // add the sBS
        selectedBS.push_back(bsGridMap[i][j]);
        tree.push_back(curConnection);  // add the logical link into the tree topology
        connections[mBSId].push_back(bsGridMap[i][j]);  // add the sBS to the 'neighbor' of mBS
        connections[bsGridMap[i][j]].push_back(mBSId);
        std::vector<Point_t> curBSPair;
        curBSPair.push_back(bsSet[mBSId]);
        curBSPair.push_back(bsSet[bsGridMap[i][j]]);
        bsPairs.push_back(curBSPair);
      }
    }
  }

  while (selectedBS.size() < bsSet.size()) {
    int numSelectedBS = (int) selectedBS.size();
    nextRoot = selectedBS[nextRootIndex];
    int rootX = bsLocation[nextRoot][0];
    int rootY = bsLocation[nextRoot][1];
    for (int i = rootX - 1; i <= rootX + 1; i++) {
      if (i < 0 || i > bsGridMap[0].size() - 1) continue;
      for (int j = rootY - 1; j <= rootY + 1; j++) {
        if (j < 0 || j > bsGridMap.size() - 1) continue;
        if (bsGridMap[i][j] < 0) continue;  // no bs at the grid
        auto it = std::find(selectedBS.begin(),selectedBS.end(),bsGridMap[i][j]);
        if (it != selectedBS.end()) continue;  // the bs has been selected
        // connect nextRoot to this bs;
        std::vector<int> curConnection;
        curConnection.push_back(nextRoot);
        curConnection.push_back(bsGridMap[i][j]);
        selectedBS.push_back(bsGridMap[i][j]);
        tree.push_back(curConnection);
        connections[nextRoot].push_back(bsGridMap[i][j]);
        connections[bsGridMap[i][j]].push_back(nextRoot);
        std::vector<Point_t> curBSPair;
        curBSPair.push_back(bsSet[nextRoot]);
        curBSPair.push_back(bsSet[bsGridMap[i][j]]);
        bsPairs.push_back(curBSPair);
        break;
      }
      if (selectedBS.size() > numSelectedBS) break;
    }
    nextRootIndex++;
    /*
     * The following code handles the case where there are still unselected BSs which are cannot be connected by the
     * non-rooted bs.
     */
    if (nextRootIndex == selectedBS.size()) {
      // find an unselected bs.
      for (int i = 0; i < bsSet.size(); i++) {
        if (std::find(selectedBS.begin(),selectedBS.end(),i) != selectedBS.end()) continue;
        // i is an unselected BS.
        int iX = bsLocation[i][0];
        int iY = bsLocation[i][1];
        if (parameters.limitMacroCell) {
            if (bsGridMap[iX][iY] < 0) continue;
        }
        for (int j = iX - 1; j <= iX + 1; j++) {
          if (j < 0 || j > bsGridMap[0].size() - 1) continue;
          for (int k = iY - 1; k <= iY + 1; k++) {
            if (k < 0 || k > bsGridMap.size() - 1) continue;
            if (bsGridMap[j][k] < 0) continue;
            auto it = std::find(selectedBS.begin(), selectedBS.end(), bsGridMap[j][k]);
            if (it != selectedBS.end()) {
              std::vector<int> curConnection;
              curConnection.push_back(bsGridMap[j][k]);
              curConnection.push_back(i);
              selectedBS.push_back(i);
              tree.push_back(curConnection);
              connections[i].push_back(bsGridMap[j][k]);
              connections[bsGridMap[j][k]].push_back(i);
              std::vector<Point_t> curBSPair;
              curBSPair.push_back(bsSet[bsGridMap[j][k]]);
              curBSPair.push_back(bsSet[i]);
              bsPairs.push_back(curBSPair);
              break;
            }
          }
          if (selectedBS.size() > numSelectedBS) break;
        }
        if (selectedBS.size() > numSelectedBS) break;
      }
    }
    if (selectedBS.size() == numSelectedBS && nextRootIndex == selectedBS.size()) break;
  }

  cout << "The number of connected BS is " << selectedBS.size() << endl;
}

void writeTopologyToFile(std::string& dataTopology, const std::vector<std::vector<int>>& connections, int& numRelays) {
  ofstream outFile;
  outFile.open(dataTopology, std::ios_base::trunc);
  if (!outFile.is_open()) {
    cout << "(E) Failed to open the file where topology information should be stored!" << endl;
  } else {
    cout << "(*) Ready to write the topology information into file." << endl;
    for (auto connection : connections) {
      outFile << connection[0] + numRelays << "\t" << connection[1] + numRelays << "\n";
    }
  }
  outFile.close();
}

void selectRelayPerGrid(std::vector<Point_t>& relays, SystemParameters& parameters){
  /* Number of grids. */
  auto numGridAlongX = (unsigned int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
  auto numGridAlongY = (unsigned int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
  /* Initialize the Map for indicating whether there is a BS in the grid. */
  std::vector<unsigned int> gridHasRelay;
  for (int i = 0; i < numGridAlongX; i++){
    for (int j = 0; j < numGridAlongY; j++){
      gridHasRelay.push_back(0);
    }
  }
  /* Iterating each candidate relay. */
  unsigned int i = 0;
  while(i < relays.size()){
    // System.out.println(i);
    Point_t currentRelay = (relays.at(i));
    auto gridIndexX = (unsigned int) floor((currentRelay.getX() - parameters.areaXRange_m[0])/parameters.gridSize_m);
    auto gridIndexY = (unsigned int) floor((currentRelay.getY() - parameters.areaYRange_m[0])/parameters.gridSize_m);
    if (gridHasRelay.at(gridIndexX*numGridAlongY + gridIndexY) >= parameters.maxNumRelaysInGrid){
      relays.erase(relays.begin() + i);
    } else {
      (gridHasRelay.at(gridIndexX*numGridAlongY + gridIndexY))++;
//      StdDraw.point(currentRelay.x, currentRelay.y);
      i++;
    }
  }
  cout << "(5) There are " + to_string(relays.size()) + " candidate roof top relays being selected." << endl;
}

void countRelaysPerGrid(std::vector<Point_t>& relays, std::vector<std::vector<int>>& numRelaysInGrid, SystemParameters& parameters){
  /* Number of grids. */
  auto numGridAlongX = (unsigned int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
  auto numGridAlongY = (unsigned int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
  /* Initialize the Map for indicating whether there is a BS in the grid. */
  for (int i = 0; i < numGridAlongX; i++){
    std::vector<int> gridPerRow;
    for (int j = 0; j < numGridAlongY; j++){
      gridPerRow.push_back(0);
    }
    numRelaysInGrid.push_back(gridPerRow);
  }
  /* Iterating each candidate relay. */
  for (auto relay : relays){
    auto gridIndexX = (unsigned int) floor((relay.getX() - parameters.areaXRange_m[0])/parameters.gridSize_m);
    auto gridIndexY = (unsigned int) floor((relay.getY() - parameters.areaYRange_m[0])/parameters.gridSize_m);
    numRelaysInGrid[gridIndexX][gridIndexY]++;

  }
  cout << "===================== The number of relays in every grid ====================" << endl;
  for (int j = 0; j < numGridAlongX; j++){
    for (int k = 0; k < numGridAlongY; k++) {
      if (numRelaysInGrid[j][k] == 0) {
        cout << "NULL\t";
      } else if (numRelaysInGrid[j][k] < 10) {
        cout << "0" << numRelaysInGrid[j][k] << "Rs\t";
      } else {
        cout << numRelaysInGrid[j][k] << "Rs\t";
      }
    }
    cout << endl;
  }
  cout << "-----------------------------------------------------------------------------" << endl;
}

void collectAllRelays(std::vector<Point_t>& allRelays, const std::vector<Building_t>& buildings,
                      const std::string& fileDataRelays) {
  for (const Building_t& bldg : buildings){
    std::vector<Point_t> curRelays = bldg.getRelays();
    allRelays.insert(allRelays.begin(), curRelays.begin(), curRelays.end());
  }

  /* Write the relays collected out into the file. */
  std::ofstream fileOut;
  fileOut.open(fileDataRelays, std::ios_base::trunc);
  if (fileOut.is_open()){
    cout << "Ready to write relays' information to file." << endl;
  } else {
    cout << "Fail to open the file where relays' information should be stored." << endl;
  }
  for (auto relay : allRelays) {
    fileOut << relay.toStringData() << "\n";
  }
  fileOut.close();
  cout << "(*) There are " + to_string(allRelays.size()) + " candidate relays generated on the surfaces of buildings."
       << endl;
}

void readNodeInfoFromFile(std::vector<Point_t>& nodes, const std::string& fileDataNodes, std::string& type){
  std::ifstream fileIn(fileDataNodes);
  std::string str;
  while (std::getline(fileIn, str))
  {
    /* str stores the string version of a node's coordination. */
    size_t pos = 0;
    std::string token;
    std::vector<double> data;
    while (true) {
      pos = str.find(',');
      if (pos != std::string::npos) {
        token = str.substr(0, pos);
        data.push_back(std::stod(token));
        str = str.substr(pos + 1);
      } else if (str.length() > 0) {
        data.push_back(std::stod(str));
        break;
      }
    }
    assert(data.size() == 3);
    Point_t node(data[0], data[1], data[2]);
    nodes.push_back(node);
  }
  cout << "(*) Node information loaded! " << nodes.size() << " " << type << "s are read." << endl;
}

void exploreConnectivity(std::vector<std::vector<int>>& neighborList,
                         const std::vector<Point_t>& nodes, const std::vector<Building_t>& buildings,
                         const std::string& fileRelayNeighbors){
  /* Configure the output file. */
  std::ofstream fileOut;
  fileOut.open(fileRelayNeighbors, std::ios_base::app);
  if (fileOut.is_open()){
    cout << "Ready to write neighbors information to file." << endl;
  } else {
    cout << "Fail to open the file where neighbors information should be stored." << endl;
  }
  /* Iterate through all nodes, find the neighbors of each node. */
  for (int i = 0; i < nodes.size(); i++) {
    std::vector<int> curRelayNeighbors = searchNonBlockLink(buildings, (nodes.at(i)), nodes);
    neighborList.push_back(curRelayNeighbors);
    std::string outputToFile = "";
    for (auto neighbor : curRelayNeighbors){
      outputToFile += std::to_string(neighbor) + "\t";
    }
    fileOut << outputToFile + "\n";
    cout << "The No.\t" + std::to_string(i) + "\tnode has\t" + std::to_string(neighborList[i].size()) + "\tnon-block neighbors." << endl;
  }
  fileOut.close();
}

void collectPhysicalLinks(std::vector<std::vector<int>>& phyLinkSet, const std::vector<std::vector<int>>& neighborList,
                          const std::vector<Point_t>& nodes, const vector<int>& selectedGrids,
                          const SystemParameters& parameters, std::string& dataPhyLinks, bool& write){
  /* Number of grids. */
  auto numGridAlongX = (unsigned int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
  auto numGridAlongY = (unsigned int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
  /* Iterate through all nodes. */
  std::vector<std::vector<int>> phyLinkSet2;
  int count = 0;
  for (int i = 0; i < neighborList.size(); ++i) {
    // nodes[i] stores the coordinates of the i-th node. Test to make sure that nodes[i] is within the seleted area.
    auto gridIndexX = (unsigned int) floor((nodes[i].getX() - parameters.areaXRange_m[0])/parameters.gridSize_m);
    auto gridIndexY = (unsigned int) floor((nodes[i].getY() - parameters.areaYRange_m[0])/parameters.gridSize_m);
    int gridId = gridIndexY * numGridAlongX + gridIndexX;  // X is column, Y is row
    if (std::find(selectedGrids.begin(), selectedGrids.end(),gridId) == selectedGrids.end()) continue;
    count++;
    for (int j = 0; j < neighborList[i].size(); ++j) {
      // neighborList[i][j] stores the index of the j-th LoS neighbor of the i-th node
      int neighborId = neighborList[i][j];
      if (neighborId <= i) continue;
      gridIndexX = (unsigned int) floor((nodes[neighborId].getX() - parameters.areaXRange_m[0])/parameters.gridSize_m);
      gridIndexY = (unsigned int) floor((nodes[neighborId].getY() - parameters.areaYRange_m[0])/parameters.gridSize_m);
      gridId = gridIndexY * numGridAlongX + gridIndexX;
      if (std::find(selectedGrids.begin(), selectedGrids.end(),gridId) == selectedGrids.end()) continue;
      if (nodes[i].distanceTo(nodes[neighborList[i][j]]) > parameters.phyLinkDistMax_m) continue;
      std::vector<int> curPhyLink;
      curPhyLink.push_back(i);
      curPhyLink.push_back(neighborList[i][j]);
      phyLinkSet.push_back(curPhyLink);
      std::vector<int> reverseLink;
      reverseLink.push_back(neighborList[i][j]);
      reverseLink.push_back(i);
      phyLinkSet2.push_back(reverseLink);
    }
  }
  phyLinkSet.insert(phyLinkSet.end(), phyLinkSet2.begin(), phyLinkSet2.end());
  cout << "(*) There are " << phyLinkSet2.size() << " connections between " << count
       << " nodes in the selected area." << endl;
  cout << "    In total, there are " << phyLinkSet.size() << " physical links." << endl;

  std::ofstream outFile;
  outFile.open(dataPhyLinks, std::ios_base::app);
  if (!outFile.is_open()) {
    cout << "(E) Failed to open the file where physical links information should be stored." << endl;
  } else if (write) {
    cout << "(O) Ready to write physical links information into file." << endl;
    for (auto link : phyLinkSet) {
      outFile << link[0] << "\t" << link[1] << "\n";
    }
  } else {
    cout << "(S) The physical link information has been stored. Skip the operation of write to file." << endl;
  }
  outFile.close();

}

void collectConsecutiveLinkPairs(std::vector<int>& consecLinkPairSet,
                                 const std::vector<std::vector<int>>& phyLinkSet,
                                 std::string& dataConsecLinkPairs, bool& write) {
  /* Configure the output file. */
  std::ofstream fileOut;
  fileOut.open(dataConsecLinkPairs, std::ios_base::app);
  if (write) {
    if (fileOut.is_open()){
      cout << "Ready to write consecutive link pairs' information to file." << endl;
    } else {
      cout << "Fail to open the file where consective link pairs' information should be stored." << endl;
    }
  } else {
    cout << "(*) Consecutive link pairs' information exists. No need to write to file." << endl;
  }


  int indicator = 0;
  int count = 0;
  for (int i = 0; i < phyLinkSet.size(); ++i) {
    for (int j = 0; j < phyLinkSet.size(); ++j) {
      if (phyLinkSet[i][1] == phyLinkSet[j][0] && phyLinkSet[i][0] != phyLinkSet[j][1]) {
        indicator = 1;
        count++;
      } else {
        indicator = 0;
      }
      consecLinkPairSet.push_back(indicator);
      if (write) {
        fileOut << indicator << "\t";
      }
    }
    if (write) {
      fileOut << "\n";
    }
  }

  cout << "(*) In total, there are " << count << " pairs of consecutive link pairs." << endl;

  fileOut.close();
}

void collectFirstLastHopCandidatePhyLinks(std::vector<std::vector<int>>& firstHopSet,
                                          std::vector<std::vector<int>>& lastHopSet,
                                          const std::vector<std::vector<int>>& phyLinkSet,
                                          const std::vector<std::vector<int>>& connectionList,
                                          int& numRelays, bool& write, std::string& dataFirstHop,
                                          std::string& dataLastHop) {
  std::ofstream outFileFirstHop, outFileLastHop;
  outFileFirstHop.open(dataFirstHop, std::ios_base::app);
  outFileLastHop.open(dataLastHop, std::ios_base::app);
  for (int i = 0; i < connectionList.size(); i++) {
    int srcId = connectionList[i][0] + numRelays;
    int dstId = connectionList[i][1] + numRelays;
    for (int j = 0; j < phyLinkSet.size(); j++) {
      if (phyLinkSet[j][0] == srcId) {
        firstHopSet[i][j] = 1;
      } else {
        firstHopSet[i][j] = 0;
      }
      if (phyLinkSet[j][1] == dstId) {
        lastHopSet[i][j] = 1;
      } else {
        lastHopSet[i][j] = 0;
      }
      if (write) {
        outFileFirstHop << firstHopSet[i][j] << "\t";
        outFileLastHop << lastHopSet[i][j] << "\t";
      }
    }
    if (write) {
      outFileFirstHop << "\n";
      outFileLastHop << "\n";
    }
  }
  outFileFirstHop.close();
  outFileLastHop.close();
}

std::vector<int> searchNonBlockLink(const std::vector<Building_t>& buildings, const Point_t& s, const std::vector<Point_t>& nodes){
  std::vector<int> nonBlockNodes;
  for(unsigned int i = 0; i < nodes.size(); i++){
    // for each node in the topology, determine whether there is a non-blocked path between s and nodes[i]
    double dist = nodes.at(i).distanceTo(s); // the distance between s and nodes[i]
    if(dist >= 1){
      // the link length is valid; otherwise that relays[i] will not be added to the list
      Line_t sd(s, (nodes.at(i)));
      bool blockTest = blockageTest(buildings, sd);
      if(!blockTest){
        // there is no blockage between s and relays[i]
        nonBlockNodes.push_back(i); // add the index of node to the list
      }
    }
  }
  return nonBlockNodes;
}

bool blockageTest(const std::vector<Building_t>& buildingSet, const Line_t& sd){
  Point_t onfacePoint(-10000.0,-10000.0,-10000.0);
  Point_t s = sd.getSrc();
  Point_t d = sd.getDst();
  bool testResult = false; // by default, false means no blockages
  /*
   * The rule is:
   * If a line segment intersect with any face of any building, and the intersection point is not the src or the dst,
   *     this line segment is blocked by this building.
   * If a line segment's src or dst is on any face of any building, the conditions are complicated, but we can determine
   *     the condition through multi-case discussion.
   */
  /* === Iterate each building === */
  for(unsigned int i = 0; i < buildingSet.size(); i++){
    Building_t curBldg = buildingSet.at(i);
    std::vector<Point_t> curVgs = curBldg.getVgs();
    std::vector<Point_t> curVts = curBldg.getVts();
    /* --- Initialize the local variables used to determine the status. --- */
    int faceIntersectCount = 0;
    int sdOnEdgeCount = 0;
    bool topfaceIC = false;
    bool sIsTopV = false;        // true: s is one of the top vertices of the building.
    bool dIsTopV = false;        // true: d is one of the top vertices of the building.
    Point_t testIntersect;   // A Point_t object to store the intersection point if it exists.
    /* --- Determine whether s or d is one of the top vertices of this building. --- */
    if(s.sameAs((curVts.at(0))) || s.sameAs((curVts.at(1)))
       || s.sameAs((curVts.at(2))) || s.sameAs((curVts.at(3)))){
      sIsTopV = true;
    }
    if(d.sameAs((curVts.at(0))) || d.sameAs((curVts.at(1)))
       || d.sameAs((curVts.at(2))) || d.sameAs((curVts.at(3)))){
      dIsTopV = true;
    }
    /* --- Test whether the line segement sd intersects with each face of the building. --- */
    /* ------ plane1 is one of the side face of the building. ------ */
    Plane_t plane1((curVgs.at(0)), (curVgs.at(1)), (curVts.at(1)), (curVts.at(0)));
    testIntersect = plane1.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      /* sd has one intersection with this face. */
      /* Test whether the intersection point is s or d. */
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        /* The intersection point is neither s or d of the line segment, thus there is blockage! */
        return true;
      } else{
        /* s or d is the intersection point. Note that, s and d cannot be the same point. */
        faceIntersectCount++; // This face intersect with sd
        /* Test whether this intersection point is on the edge or not. */
        if(std::abs(testIntersect.getZ() - curVts.at(1).getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(1).getX()) + std::abs(testIntersect.getY() - curVts.at(1).getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(0).getX()) + std::abs(testIntersect.getY() - curVts.at(0).getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t plane2((curVgs.at(1)), (curVgs.at(2)), (curVts.at(2)), (curVts.at(1)));
    testIntersect = plane2.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect.getZ() - curVts.at(1).getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(1).getX()) + std::abs(testIntersect.getY() - curVts.at(1).getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(2).getX()) + std::abs(testIntersect.getY() - curVts.at(2).getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t plane3((curVgs.at(2)), (curVgs.at(3)), (curVts.at(3)), (curVts.at(2)));
    testIntersect = plane3.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect.getZ() - curVts.at(2).getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(2).getX()) + std::abs(testIntersect.getY() - curVts.at(2).getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(3).getX()) + std::abs(testIntersect.getY() - curVts.at(3).getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t plane4((curVgs.at(3)), (curVgs.at(0)), (curVts.at(0)), (curVts.at(3)));
    testIntersect = plane4.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect.getZ() - curVts.at(0).getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(3).getX()) + std::abs(testIntersect.getY() - curVts.at(3).getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect.getX() - curVts.at(0).getX()) + std::abs(testIntersect.getY() - curVts.at(0).getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t plane5((curVts.at(0)), (curVts.at(1)), (curVts.at(2)), (curVts.at(3)));
    testIntersect = plane5.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        return true;
      } else{
        faceIntersectCount++;
        Vector_t AI((curVts.at(0)), testIntersect);
        Vector_t AB((curVts.at(0)), (curVts.at(1)));
        Vector_t AD((curVts.at(0)), (curVts.at(3)));
        double Ix = AB.dot(AI)/AB.mod();  // the x value of p
        double Iy = AD.dot(AI)/AD.mod();  // the y value of p
        if(std::abs(Ix) <= 1E-5 || std::abs(Ix-AB.mod()) <= 1E-5 || std::abs(Iy) <= 1E-5 || std::abs(Iy-AD.mod()) <= 1E-5){
          sdOnEdgeCount++;
        }
      }
    }
    Plane_t plane6((curVgs.at(0)), (curVgs.at(1)), (curVgs.at(2)), (curVgs.at(3)));
    testIntersect = plane6.planeIntersectLine(sd);
    if(testIntersect.getValid()){
      if(!(testIntersect.sameAs(s) || testIntersect.sameAs(d))){
        return true;
      } else{
        faceIntersectCount++;
      }
    }
    /* === Determine the status of the blockage. === */
    if(faceIntersectCount > 5){
      cout << "Error!!! The line has more than 5 intersections with a building." << endl;
    }
    if(faceIntersectCount == 5){
      /*
       * When faceIntersectionCount == 5,
       * The only possible case: either s or d is one vertex, and the other is on the edge intersected by two other faces. There is blockage.
       */
      return true;
    }
    if(faceIntersectCount == 4){
      /*
       * When faceIntersectCount == 4, both s and d are on this building.
       * (1) s or d is top vertex, and the other is on the faces not intent with that vertex, blocked!
       * (2) s and d are on two edges not in the same face, blocked!
       */
      if (!(sIsTopV && dIsTopV)){
        // not the case (two top vertices on different edges: no blockage)
        return true;
      }
    }
    if(faceIntersectCount == 3){
      if((!sIsTopV) && (!dIsTopV)){
        return true;
      }
    }
    if(faceIntersectCount == 2){
      if((!sIsTopV) && (!dIsTopV)){
        if(sdOnEdgeCount == 0){
          return true;
        }
      }
    }
  }

  /*
   * (1) When faceIntersectCount == 1, the only case will be a point in the middle of a face and ejects a line out towards other buildings.
   * Thus, there is no blockage.
   * When faceIntersectCount == 2,
   *   (2) If the two intersection points are the same point,
   *     It means the point is on the edge of the building, and it can only eject a line out towards other buildings,
   *     thus, there is no blockage.
   *   Else the two intersection points are different points,
   *     If these two points are on the same face of the building, then neither of them is on the edge, otherwise, there will be at least 3 points.
   *     However, if both of them are within the face, the line is parallel to the face, there is no intersection.
   *     (3) Thus, these two points must be in different faces. And there must be blockage.
   * When faceIntersectCount == 3,
   *   (4) If all of three points are the same, it is a vertex of the building and there is no blockage.
   *   (5) If two of them are the same, then it must be on one edge of the building. If the other point is on either of the two faces intented with the previous node, there is only one intersection point. Thus, in this case, there is blockage.
   *   If all of them are different, it is impossible.
   * When faceIntersectionCount == 6,
   *   Only two diagnoal vertices can be that case, blockage. However, since relays or base stations cannot be deployed on the ground, this is impossible.
   */


  return false;
}


void getRelayNeighborInfoFromFile(std::vector<std::vector<int>>& relayNeighborList, std::string dataRelayNeighbors){
  std::ifstream fileIn(dataRelayNeighbors);
  std::string str;
  while (std::getline(fileIn, str)) {
    std::vector<int> data;
    if (str.length() == 0) {
      relayNeighborList.push_back(data);
      continue;
    }
    while (str.find('\t') != std::string::npos) {
      // as long as the str contains '\t'
      std::string front = str.substr(0, str.find('\t'));
      data.push_back(std::stoi(front));
      str = str.substr(str.find('\t') + 1);
    }
    if (str.length() > 0) {
      data.push_back(std::stoi(str));
    }
    relayNeighborList.push_back(data);
  }
//  while (std::getline(fileIn, str, '\t'))
//  {
//    if (str.find('\n') != std::string::npos){
////      data.push_back(str.substr(0, str.find('\n')));
//      relayNeighborList.push_back(*data);
//      data = new std::vector<int>();
//      std::string temp = str.substr(str.find('\n')+1);
//      if (temp != "") {
//        data->push_back(std::stoi(str.substr(str.find('\n')+1)));
//      }
//    } else {
//      data->push_back(std::stoi(str));
//    }
//  }

  cout << "(*) Node connectivity information loaded!" << endl;
}


std::vector<std::vector<Point_t>> generateBaseStationPairs(const std::vector<Point_t>& bsSet,
                                                           SystemParameters& parameters){
  /* Set up random generator. */
  // Seed with a real random value, if available
  std::random_device r;
  std::default_random_engine e(r());
  std::uniform_int_distribution<int> uniform_dist(0, bsSet.size()-1);
  /* Initialize the (source,destination) pair */
  std::vector<std::vector<Point_t>> sdPair((unsigned) parameters.numBSPairs, std::vector<Point_t>(2));
  // This is a function to generate different base station pairs to complete the simulation, which is very useful
  /* Initialize counter */
  int countPair = 0;
  double lowerBound = parameters.bsDistanceRange_m[0];
  double upperBound = parameters.bsDistanceRange_m[1];
  while(countPair<100) {
    Point_t srcRnd = bsSet.at((unsigned) uniform_dist(e));
    Point_t dstRnd = bsSet.at((unsigned) uniform_dist(e));
    double distTemp = srcRnd.distanceTo(dstRnd);
    if(distTemp > lowerBound && distTemp <= upperBound){
      sdPair[(unsigned) countPair][0] = srcRnd;
      sdPair[(unsigned) countPair][1] = dstRnd;
      countPair++;
    }
  }
  return sdPair;
}

std::vector<std::vector<int>> addNodeToConnectivityList(const std::vector<std::vector<int>>& relayNeighborList,
                                                        const Point_t& newNode, const std::vector<Point_t>& oldNodes,
                                                        const std::vector<Building_t>& buildings){
  std::vector<std::vector<int>> currentList = relayNeighborList;
  /* Vector which stores the index of nodes which are connected to the new node */
  std::vector<int> nonBlockNodes;
  /* Test the connection between the new node and each of the old node. */
  for (int i = 0; i < oldNodes.size(); i++){
    Line_t sd(newNode, oldNodes[i]);
    bool blockTest = blockageTest(buildings, sd);
    if (!blockTest){
      nonBlockNodes.push_back(i);
      currentList[i].push_back(oldNodes.size());
    }
  }
  currentList.push_back(nonBlockNodes);
  return currentList;
}

void writeVectorDataToFile(const string& filename, const std::vector<std::vector<int>>& data) {
    std::ofstream fileOut;
    fileOut.open(filename, std::ios_base::trunc);
    if (fileOut.is_open()) {
        for (auto row : data){
            for (auto element : row) {
                fileOut << element << '\t';
            }
            fileOut << '\n';
        }
    }
    fileOut.close();
}


void searchPathDecodeForwardMaxHop(Path_t& paths, const std::vector<Point_t>& nodes,
                                   const std::vector<std::vector<int>>& nodeNeighborList,
                                   const int& relayNum, SystemParameters& parameters,
                                   const std::map<int, std::vector<Vector_t>>& phyLinksAtBSs,
                                   const std::map<int, Vector_t>& phyLinks,
                                   const std::vector<int>& selectedRelays){
  /*
   * Get source and destination of this path searching process.
   */
  Point_t src = nodes[paths.getSrcId()];
  Point_t dst = nodes[paths.getDstId()];
  /* Initialize a path with source node id in it. */
  std::vector<int> curPath;
  curPath.push_back(paths.getSrcId());

  searchNextHopNode(paths, curPath, nodes, nodeNeighborList, relayNum, 0, paths.getMaxHopNum(), 40, 40, parameters, phyLinksAtBSs, phyLinks, selectedRelays);
}


void searchNextHopNode(Path_t& paths, const std::vector<int>& curPath, const std::vector<Point_t>& nodes,
                       const std::vector<std::vector<int>>& nodeNeighborList, const int& relayNum,
                       const int& preHopNum, const int& maxHopNum, const double& preHopCap,
                       const double& curPathThroughput, SystemParameters& parameters,
                       const std::map<int, std::vector<Vector_t>>& phyLinksAtBSs,
                       const std::map<int, Vector_t>& phyLinks,
                       const std::vector<int>& selectedRelays) {
  /* Get the source and destination node of the path. */
  Point_t src = nodes[paths.getSrcId()];
  Point_t dst = nodes[paths.getDstId()];

  /* Update the current hop number. The searching process starts from hop 0 at src. */
  int curHopNum = preHopNum + 1;
  /* If current hop exceeds the maximum hop number allowed, the searching process ends. */
  if (curHopNum > maxHopNum) {
    return;
  } else {
    /* The current hop number is valid. */
    assert(curHopNum == curPath.size());
    /* Get candidate nodes of this hop, which are the neighboring nodes of the last node in curPath. */
    int preNodeId = curPath.back();       // the last element of curPath is the index of the last selected node.
    Point_t preNode = nodes[preNodeId];   // the previous node
    std::vector<int> candidates = nodeNeighborList[preNodeId]; // The indices of all neighbors of the previous node
    /*
     * Iterates each candidate node for the next node to be inserted into the path.
     */
    for (auto candidateId : candidates) {
      /* The candidate nodes must only be destination BS and relays. */
      if (candidateId >= relayNum && candidateId != paths.getDstId()) continue;
      /* Relay sharing control */
      if (parameters.relaySharingControl) {
        if (std::find(selectedRelays.begin(), selectedRelays.end(), candidateId) != selectedRelays.end()) continue;
      }
      /* First hop control */
      if (parameters.firstHopControl && curHopNum == 1) {
        Point_t curNode = nodes[candidateId];
        Vector_t firstHop(preNode, curNode);
        if (phyLinksAtBSs.find(preNodeId) != phyLinksAtBSs.end()) {
          bool isoOK = true;
          for (auto phyLink : phyLinksAtBSs.at(preNodeId)) {
            double angle = acos(firstHop.dot(phyLink)/firstHop.mod()/phyLink.mod());
            if (angle < parameters.antennaIsoSpan_phi) {
              isoOK = false;
              break;
            }
          }
          if (!isoOK) continue;
        }
      }
      /* A valid candidate should not be selected before in curPath. */
      if (find(curPath.begin(), curPath.end(), candidateId) == curPath.end()) {
        Point_t curNode = nodes[candidateId];
        double linkLength_m = preNode.distanceTo(curNode); // The link to be selected is from preNode to curNode
        double distToDst_m = curNode.distanceTo(dst);      // The distance between curNode and dst
        /*
         * Threshold 1: distance
         */
        /* If the current link length is above threshold, this candidate node will not be selected. */
        if (linkLength_m > parameters.phyLinkDistMax_m) continue;
        /* If the distance from the candidate node to destination is too far away, it will not be selected. */
        if (distToDst_m > parameters.phyLinkDistMax_m * (maxHopNum - curHopNum)) continue;
        /*
         * Threshold 2: capacity
         */
        double curHopCap = calculateLinkCapacity_Gbps(linkLength_m, parameters);
        /* The capacity threshold only takes effect when there are more than one hop in the path. */
        double curLinkPairCap = preHopCap * curHopCap / (preHopCap + curHopCap);
        if (curHopNum > 1) {
          /* The current capacity of the link pair should be larger than the best path among all selected paths with more than 1 hops. */
          if (curLinkPairCap < paths.getMultiHopMaxThroughput()) continue;
        }
        /*
         * Threshold 3: interference
         */
        bool intraInterference = intraPathInterferenceAddLink(curPath, preNodeId, candidateId, nodes,
                                                              nodeNeighborList, parameters);
        if (intraInterference) continue;
        /* Add inter path interference detection here. */
        if (parameters.interPathIntControl) {
          bool interInterference = checkInterPathInterference(preNodeId, candidateId, phyLinks, nodes,
                                                              nodeNeighborList, parameters);
          if (interInterference) continue;
        }


        /* The searching process only continues when there is no intra path interference. */
        /*
         * The currently selected node is a valid candidate node which should be added into an updated path.
         */
        std::vector<int> updatePath = curPath;
        updatePath.push_back(candidateId);
        double updatePathThroughput = curPathThroughput;
        if (curLinkPairCap < curPathThroughput) {
          updatePathThroughput = curLinkPairCap;
        }

        /* When currently selected node is the destination node. */
        if (candidateId == paths.getDstId()) {
          if (parameters.firstHopControl) {
            Point_t curNode = nodes[candidateId];
            Vector_t firstHop(curNode, preNode);
            if (phyLinksAtBSs.find(candidateId) != phyLinksAtBSs.end()) {
              bool isoOK = true;
              for (auto phyLink : phyLinksAtBSs.at(candidateId)) {
                double angle = acos(firstHop.dot(phyLink)/firstHop.mod()/phyLink.mod());
                if (angle < parameters.antennaIsoSpan_phi) {
                  isoOK = false;
                  break;
                }
              }
              if (!isoOK) continue;
            }
          }
          /* Add this new path to path list if the path is better than the best path in the list. */
          if (curHopNum > 1 && updatePathThroughput > paths.getMultiHopMaxThroughput()) {
            paths.pathList.push_back(updatePath);
            paths.pathThroughput.push_back(updatePathThroughput);
            paths.setMultiHopMaxThroughput(updatePathThroughput);
            paths.setMultiHopMaxThroughputId(paths.pathList.size()-1);
            cout << "A new path with " << curHopNum << " hops, and throughput " << updatePathThroughput
                 << " Gbps has been found!" << endl;
          }
          if (curHopNum ==  1) {
            paths.pathList.push_back(updatePath);
            paths.pathThroughput.push_back(curHopCap);
            paths.setSingleHopMaxThroughput(curHopCap);
            paths.setSingleHopMaxThroughputId(paths.pathList.size()-1);
            cout << "A new path with 1 hop and throughput " << curHopCap << " Gbps has been found!" << endl;
          }
        } else {
          /* The currently selected node is not the destination node and the searching process continues with an updated path. */
          searchNextHopNode(paths, updatePath, nodes, nodeNeighborList, relayNum, curHopNum, maxHopNum, curHopCap,
                            updatePathThroughput, parameters, phyLinksAtBSs, phyLinks, selectedRelays);
        }
      }
      /* If the currently viewed candidate node has been selected in curPath, this node should be discarded. */
      // do nothing, the for loop continues.
    }
    // All candidates have been tested, the searching process ends.
  }
}

bool intraPathInterferenceAddLink(const std::vector<int>& path, const int& lLId, const int& lRId,
                                  const std::vector<Point_t>& nodes,
                                  const std::vector<std::vector<int>>& nodeNeighborList,
                                  const SystemParameters& parameters) {
  int pathHopNum = path.size() - 1; // the number of hops in the current path
  /*
   * As for intra path interference, the last hop in path does not need to be considerred, because that hop will never
   * transmit together with the new link.
   */
  for (int i = 0; i < (pathHopNum - 1); i++) {
    /* From left to right. However, the case from right to left is the same. */
    Point_t tempL = nodes[path[i]];
    Point_t tempR = nodes[path[i+1]];
    /* Determine whether the interference link from tempL to lR is blocked. */
    if (std::find(nodeNeighborList[path[i]].begin(), nodeNeighborList[path[i]].end(), lRId) != nodeNeighborList[path[i]].end()){
      Vector_t tempLR(tempL, tempR);
      Vector_t intLR(tempL, nodes[lRId]);
      Vector_t lLR(nodes[lLId], nodes[lRId]);
      double angTempIntLR = acos(tempLR.dot(intLR)/tempLR.mod()/intLR.mod());
      double angIntLLR = acos(intLR.dot(lLR)/intLR.mod()/lLR.mod());
      if (angTempIntLR < parameters.antennaBeamWidth_phi/2 || angIntLLR < parameters.antennaBeamWidth_phi/2) {
        /* There is interference */
        return true;
      }
    }
  }
  return false;
}


/*
 * Dijkstra algorithm.
 */
std::vector<int> Dijkstra(const std::vector<std::vector<int>>& neighborList, const std::vector<Point_t>& nodes,
                          int srcId, std::string pathFile, std::string type){
  /* Set up for drawing src and dst */
//  StdDraw.point(nodes[nodes.length-2].x,nodes[nodes.length-2].y);
//  StdDraw.point(nodes[nodes.length-1].x,nodes[nodes.length-1].y);

  /* Stores the index of unvisited nodes in nodes array. */
  std::vector<int> unvisitedSet;
  /* Distance between src and all nodes (i.e., relays+src+dst) in the topology */
  double distToSrc[nodes.size()];
  /* Index of the previous node to node[i] on the shortest path */
  int prevIndex[nodes.size()];
  /*
   * Initialization:
   * The first nodes.length-2 nodes are relay nodes, then source node,
   * and the last one is destination node.
   */
  for (int i=0; i<nodes.size(); i++){
    distToSrc[i] = 1.0E10;    // 1000000 means infinity
    prevIndex[i] = -1;     // -1 means "undefined"
    unvisitedSet.push_back(i);    // add node i into unvisited node set.
  }
  /* Distance from source to source is zero. */
  distToSrc[srcId] = 0;
//  distToSrc[nodes.size() - 2] = 0;
  /* Test each node in the unvisited set. */
  while (!unvisitedSet.empty()){
    /* Find the node in unvisitedSet with the min distToSrc[] */
    double distMin = 1.0E10;   // 10 times initial distance
    int indexOfUnvisitedSetMin = -1; // the index of the min distToSrc[] node in unvisited set
    int nodeIndexDistMin = -1;   // the index of the node in all nodes
    /* Find the min distToSrc[] */
    for (int i=0; i<unvisitedSet.size(); i++){
      if (distToSrc[unvisitedSet.at(i)] < distMin){
        indexOfUnvisitedSetMin = i;
        distMin = distToSrc[unvisitedSet.at(i)];
      }
    }

    if (indexOfUnvisitedSetMin == -1) {
      cout << "There are " + std::to_string(unvisitedSet.size()) + " nodes unvisited, but they are not reachable."
                                                                   << endl;
      break;
    }

    nodeIndexDistMin = unvisitedSet.at(indexOfUnvisitedSetMin);
    /* Remove the node of min distToSrc[] from unvisited set. */
    unvisitedSet.erase(unvisitedSet.begin() + indexOfUnvisitedSetMin);
    /* Get the LoS neighbors of the currently selected node. */
    std::vector<int> nodeNeighborList = neighborList.at(nodeIndexDistMin);

    /* Update the dist and prev of each neighbor. */
    for (int i = 0; i < nodeNeighborList.size(); i++){
      Point_t neighbor = nodes[nodeNeighborList.at(i)];
      double alt = 0.0;
      if (type == "distance"){
        alt = distToSrc[nodeIndexDistMin] + nodes[nodeIndexDistMin].distanceTo(neighbor);}
      else if (type == "hop"){
        alt = distToSrc[nodeIndexDistMin] + 1;
      } else {
        alt = distToSrc[nodeIndexDistMin] + calculateWeight(nodes[nodeIndexDistMin].distanceTo(neighbor));
      }
      if (alt < distToSrc[nodeNeighborList.at(i)]){
        distToSrc[nodeNeighborList.at(i)] = alt;
        prevIndex[nodeNeighborList.at(i)] = nodeIndexDistMin;
      }
      if (alt > 1.0E10) {
        cout << "Warning: The distance 'alt' is larger than 1.0E10." << endl;
        break;
      }
    }
  }

  std::vector<int> path;
  int currentIndex = nodes.size() - 1; // destination node
  while (currentIndex != (srcId)){
    path.push_back(currentIndex);
    if (prevIndex[currentIndex] == -1){
      cout << "There is no path to the destination, because the current node does not have previous hop." << endl;
      path.clear();
      break;
    }
//    StdDraw.line(nodes[currentIndex].x, nodes[currentIndex].y, nodes[prevIndex[currentIndex]].x, nodes[prevIndex[currentIndex]].y);
    currentIndex = prevIndex[currentIndex];
  }
  if ((!path.empty()) && (currentIndex == (nodes.size()-2))) {
    path.push_back(currentIndex);
    std::ofstream fileOutPath;
    fileOutPath.open(pathFile, std::ios_base::app);
    for (int i : path) {
      fileOutPath << nodes[i].toStringData() << "\n";

    }
    fileOutPath.close();
  }

  return path;
}

double calculateLinkCapacity_Gbps(double linkLength_m, SystemParameters& parameters){
  double pt_w = parameters.pt_w;
  double pt_dBm = 10.0 * log10(pt_w * 1000.0); // dBm = 10*log10(w*1000)
  double eirp_dBm = pt_dBm + parameters.antennaGain_dBi;
  double pathLoss_dB = parameters.exponent * 10.0 * log10(4.0 * M_PI * linkLength_m / parameters.lambda_m);
  double pr_dBm = eirp_dBm - pathLoss_dB - parameters.alpha * linkLength_m - parameters.linkMargin_dB + parameters.antennaGain_dBi;
  double sinr_dB = pr_dBm - parameters.noise_dBm;
  if (sinr_dB > 50.0) sinr_dB = 50.0;
  double sinr = pow(10.0, sinr_dB/10.0);
  double capacity_Gbps = 2.16 * log2(1 + sinr);
  return capacity_Gbps;
}

double calculateWeight(double dist){
  return exp(0.0037*dist)*dist*dist;
}

bool checkInterPathInterference(const int s1i, const int d1i, const std::map<int, Vector_t>& phyLinks,
                                const std::vector<Point_t>& nodes,
                                const std::vector<std::vector<int>>& nodeNeighborList,
                                const SystemParameters& parameters) {

  double halfBeam = parameters.antennaBeamWidth_phi/2.0;
  double isoSpan = parameters.antennaIsoSpan_phi;

  Point_t s1 = nodes[s1i];
  Point_t d1 = nodes[d1i];
  Vector_t s1d1(s1,d1);
  Vector_t d1s1(d1,s1);

  for (auto phyLink : phyLinks) {
    int s2i = phyLink.first / parameters.maxNumPhyLinks;
    int d2i = phyLink.first % parameters.maxNumPhyLinks;
    Point_t s2 = nodes[s2i];
    Point_t d2 = nodes[d2i];
    Vector_t s2d2(s2,d2);
    Vector_t d2s2(d2,s2);

    // test interference signal s1-->d2, same as d2-->s1
    if (std::find(nodeNeighborList[s1i].begin(),nodeNeighborList[s1i].end(),d2i) != nodeNeighborList[s1i].end()){
      Vector_t s1d2(s1,d2);
      if (s1d2.mod() > 0) {
        double a_d1s1d2 = acos(s1d1.dot(s1d2)/s1d1.mod()/s1d2.mod());
        double a_s1d2s2 = acos(s1d2.dot(s2d2)/s1d2.mod()/s2d2.mod());
        if (a_d1s1d2 < halfBeam || a_s1d2s2 < halfBeam) {
          if (a_d1s1d2 > 0.0001 && a_s1d2s2 > 0.0001) {
            return true;
          }
          if (a_d1s1d2 <= 0.0001 && a_s1d2s2 < isoSpan) {
            return true;
          }
          if (a_s1d2s2 <= 0.0001 && a_d1s1d2 < isoSpan) {
            return true;
          }
        }
      } else {
        double a_d1s1s2 = acos(s1d1.dot(d2s2)/s1d1.mod()/d2s2.mod());
        if (a_d1s1s2 < 2*halfBeam) {
          return true;
        }
      }
    }

    // test interference signal s2-->d1, same as d1-->s2
    if (std::find(nodeNeighborList[s2i].begin(),nodeNeighborList[s2i].end(),d1i) != nodeNeighborList[s2i].end()){
      Vector_t s2d1(s2,d1);
      if (s2d1.mod() > 0) {
        double a_s1d1s2 = acos(s1d1.dot(s2d1)/s1d1.mod()/s2d1.mod());
        double a_d1s2d2 = acos(s2d1.dot(s2d2)/s2d1.mod()/s2d2.mod());
        if (a_s1d1s2 < halfBeam || a_d1s2d2 < halfBeam) {
          if (a_s1d1s2 > 0.0001 && a_d1s2d2 > 0.0001) {
            return true;
          }
          if (a_s1d1s2 <= 0.0001 && a_d1s2d2 < isoSpan) {
            return true;
          }
          if (a_d1s2d2 <= 0.0001 && a_s1d1s2 < isoSpan) {
            return true;
          }
        }
      } else {
        double a_s1d1d2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
        if (a_s1d1d2 < 2*halfBeam) {
          return true;
        }
      }
    }

    // test interference signal d1-->d2, same as d2-->d1
    if (std::find(nodeNeighborList[d1i].begin(),nodeNeighborList[d1i].end(),d2i) != nodeNeighborList[d1i].end()){
      Vector_t d1d2(d1,d2);
      if (d1d2.mod() > 0) {
        double a_s1d1d2 = acos(-1*s1d1.dot(d1d2)/s1d1.mod()/d1d2.mod());
        double a_s2d2d1 = acos(s2d2.dot(d1d2)/s2d2.mod()/d1d2.mod());
        if (a_s1d1d2 < halfBeam || a_s2d2d1 < halfBeam) {
          if (a_s1d1d2 > 0.0001 && a_s2d2d1 > 0.0001) {
            return true;
          }
          if (a_s1d1d2 <= 0.0001 && a_s2d2d1 < isoSpan) {
            return true;
          }
          if (a_s2d2d1 <= 0.0001 && a_s1d1d2 < isoSpan) {
            return true;
          }
//            return true;
        }
      } else {
        double a_s1d1s2 = acos(d1s1.dot(d2s2)/d1s1.mod()/d2s2.mod());
        if (a_s1d1s2 < 2*halfBeam) {
          return true;
        }
      }
    }

    // test interference signal s2-->s1, same as s1-->s2
    if (std::find(nodeNeighborList[s1i].begin(),nodeNeighborList[s1i].end(),s2i) != nodeNeighborList[s1i].end()){
      Vector_t s1s2(s1,s2);
      if (s1s2.mod() > 0) {
        double a_s2s1d1 = acos(s1s2.dot(s1d1)/s1s2.mod()/s1d1.mod());
        double a_d2s2s1 = acos(-1 * s2d2.dot(s1s2)/s2d2.mod()/s1s2.mod());
        if (a_s2s1d1 < halfBeam || a_d2s2s1 < halfBeam) {
          if (a_s2s1d1 > 0.0001 && a_d2s2s1 > 0.0001) {
            return true;
          }
          if (a_s2s1d1 <= 0.0001 && a_d2s2s1 < isoSpan) {
            return true;
          }
          if (a_d2s2s1 <= 0.0001 && a_s2s1d1 < isoSpan) {
            return true;
          }
        }
      } else {
        double a_d1s1d2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
        if (a_d1s1d2 < 2*halfBeam) {
          return true;
        }
      }
    }
  }

  return false;

}

bool checkTwoPathsInterference(const std::vector<int>& path1, const std::vector<int>& path2,
                               const std::vector<Point_t>& sd1, const std::vector<Point_t>& sd2,
                               const std::vector<std::vector<int>>& nodeNeighborList,
                               const std::vector<Building_t>& buildings,
                               const std::vector<Point_t>& nodes, const SystemParameters& parameters) {
  int hop1 = path1.size() - 1;
  int hop2 = path2.size() - 1;
  double halfBeam = parameters.antennaBeamWidth_phi/2.0;
  double isoSpan = parameters.antennaIsoSpan_phi;

  for (int i = 0; i < hop1; i++) {
    Point_t s1 = nodes[path1[i]];
    int s1i = path1[i];
    Point_t d1 = nodes[path1[i+1]];
    int d1i = path1[i+1];

    Vector_t s1d1(s1,d1);
    Vector_t d1s1(d1,s1);
    for (int j = 0; j < hop2; j++) {
      Point_t s2 = nodes[path2[j]];
      int s2i = path2[j];
      Point_t d2 = nodes[path2[j+1]];
      int d2i = path2[j+1];

      Vector_t s2d2(s2,d2);
      Vector_t d2s2(d2,s2);

      // test interference signal s1-->d2, same as d2-->s1
      if (std::find(nodeNeighborList[s1i].begin(),nodeNeighborList[s1i].end(),d2i) != nodeNeighborList[s1i].end()){
        Vector_t s1d2(s1,d2);
        if (s1d2.mod() > 0) {
          double a_d1s1d2 = acos(s1d1.dot(s1d2)/s1d1.mod()/s1d2.mod());
          double a_s1d2s2 = acos(s1d2.dot(s2d2)/s1d2.mod()/s2d2.mod());
          if (a_d1s1d2 < halfBeam || a_s1d2s2 < halfBeam) {
            if (a_d1s1d2 > 0.0001 && a_s1d2s2 > 0.0001) {
              return true;
            }
            if (a_d1s1d2 <= 0.0001 && a_s1d2s2 < isoSpan) {
              return true;
            }
            if (a_s1d2s2 <= 0.0001 && a_d1s1d2 < isoSpan) {
              return true;
            }
          }
        } else {
          double a_d1s1s2 = acos(s1d1.dot(d2s2)/s1d1.mod()/d2s2.mod());
          if (a_d1s1s2 < 2*halfBeam) {
            return true;
          }
        }
      }

      // test interference signal s2-->d1, same as d1-->s2
      if (std::find(nodeNeighborList[s2i].begin(),nodeNeighborList[s2i].end(),d1i) != nodeNeighborList[s2i].end()){
        Vector_t s2d1(s2,d1);
        if (s2d1.mod() > 0) {
          double a_s1d1s2 = acos(s1d1.dot(s2d1)/s1d1.mod()/s2d1.mod());
          double a_d1s2d2 = acos(s2d1.dot(s2d2)/s2d1.mod()/s2d2.mod());
          if (a_s1d1s2 < halfBeam || a_d1s2d2 < halfBeam) {
            if (a_s1d1s2 > 0.0001 && a_d1s2d2 > 0.0001) {
              return true;
            }
            if (a_s1d1s2 <= 0.0001 && a_d1s2d2 < isoSpan) {
              return true;
            }
            if (a_d1s2d2 <= 0.0001 && a_s1d1s2 < isoSpan) {
              return true;
            }
          }
        } else {
          double a_s1d1d2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
          if (a_s1d1d2 < 2*halfBeam) {
            return true;
          }
        }
      }

      // test interference signal d1-->d2, same as d2-->d1
      if (std::find(nodeNeighborList[d1i].begin(),nodeNeighborList[d1i].end(),d2i) != nodeNeighborList[d1i].end()){
        Vector_t d1d2(d1,d2);
        if (d1d2.mod() > 0) {
          double a_s1d1d2 = acos(-1*s1d1.dot(d1d2)/s1d1.mod()/d1d2.mod());
          double a_s2d2d1 = acos(s2d2.dot(d1d2)/s2d2.mod()/d1d2.mod());
          if (a_s1d1d2 < halfBeam || a_s2d2d1 < halfBeam) {
            if (a_s1d1d2 > 0.0001 && a_s2d2d1 > 0.0001) {
              return true;
            }
            if (a_s1d1d2 <= 0.0001 && a_s2d2d1 < isoSpan) {
              return true;
            }
            if (a_s2d2d1 <= 0.0001 && a_s1d1d2 < isoSpan) {
              return true;
            }
//            return true;
          }
        } else {
          double a_s1d1s2 = acos(d1s1.dot(d2s2)/d1s1.mod()/d2s2.mod());
          if (a_s1d1s2 < 2*halfBeam) {
            return true;
          }
        }
      }

      // test interference signal s2-->s1, same as s1-->s2
      if (std::find(nodeNeighborList[s1i].begin(),nodeNeighborList[s1i].end(),s2i) != nodeNeighborList[s1i].end()){
        Vector_t s1s2(s1,s2);
        if (s1s2.mod() > 0) {
          double a_s2s1d1 = acos(s1s2.dot(s1d1)/s1s2.mod()/s1d1.mod());
          double a_d2s2s1 = acos(-1 * s2d2.dot(s1s2)/s2d2.mod()/s1s2.mod());
          if (a_s2s1d1 < halfBeam || a_d2s2s1 < halfBeam) {
            if (a_s2s1d1 > 0.0001 && a_d2s2s1 > 0.0001) {
              return true;
            }
            if (a_s2s1d1 <= 0.0001 && a_d2s2s1 < isoSpan) {
              return true;
            }
            if (a_d2s2s1 <= 0.0001 && a_s2s1d1 < isoSpan) {
              return true;
            }
          }
        } else {
          double a_d1s1d2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
          if (a_d1s1d2 < 2*halfBeam) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void recordPhysicalLinksInAPath(std::map<int, Vector_t>& allPhysicalLinks, const std::vector<int>& path,
                                const std::vector<Point_t>& nodes, const SystemParameters& parameters){
  int numHops = path.size()-1;
  /* Iterate each hop in the path. */
  for (int i = 0; i < numHops; i++) {
    int srcId = path[i];
    int dstId = path[i+1];
    int phyLinkId = srcId * parameters.maxNumPhyLinks + dstId;
    Point_t src = nodes[srcId];
    Point_t dst = nodes[dstId];
    Vector_t sd(src, dst);
    if (allPhysicalLinks.find(phyLinkId) == allPhysicalLinks.end()) {
      /* The physical link is a new one. */
      allPhysicalLinks.insert(std::pair<int, Vector_t>(phyLinkId, sd));
    } else {
      std::cout << "Warning! A duplicated physical link has been selected!" << std::endl;
    }

    phyLinkId = dstId * parameters.maxNumPhyLinks + srcId;
    Vector_t ds(src, dst);
    if (allPhysicalLinks.find(phyLinkId) == allPhysicalLinks.end()) {
      /* The physical link is a new one. */
      allPhysicalLinks.insert(std::pair<int, Vector_t>(phyLinkId, ds));
    } else {
      std::cout << "Warning! A duplicated physical link has been selected!" << std::endl;
    }
  }
}

void recordRelaysInAPath(std::vector<int>& allRelaysSelected, const std::vector<int>& path,
                         const std::vector<Point_t>& nodes, const SystemParameters& parameters){
    int numHops = path.size()-1;
    /* Iterate each hop in the path. */
    for (int i = 1; i < numHops; i++) {
        if (std::find(allRelaysSelected.begin(), allRelaysSelected.end(), path[i]) == allRelaysSelected.end()) {
            allRelaysSelected.push_back(path[i]);
        } else {
            cout << "(W) A duplicated relay has been used." << endl;
        }
    }
}

void collectPhyLinksAtBSs(std::map<int, std::vector<Vector_t>>& phyLinksAtBSs, const std::vector<int>& path,
                          const std::vector<Point_t>& nodes) {
  int numHops = path.size() - 1;
  int srcId = path.front();
  int secNodeId = path[1];
  Point_t src = nodes[srcId];
  Point_t secNode = nodes[secNodeId];
  Vector_t firstHop(src, secNode);
  if (phyLinksAtBSs.find(srcId) == phyLinksAtBSs.end()) {
    std::vector<Vector_t> curBSLinks;
    curBSLinks.push_back(firstHop);
    phyLinksAtBSs.insert(std::pair<int, std::vector<Vector_t>>(srcId, curBSLinks));
  } else {
    phyLinksAtBSs.at(srcId).push_back(firstHop);
  }

  int dstId = path.back();
  int secLastId = path[numHops - 1];
  Point_t dst = nodes[dstId];
  Point_t secLast = nodes[secLastId];
  Vector_t lastHop(dst, secLast);
  if (phyLinksAtBSs.find(dstId) == phyLinksAtBSs.end()) {
    std::vector<Vector_t> curBSLinks;
    curBSLinks.push_back(lastHop);
    phyLinksAtBSs.insert(std::pair<int, std::vector<Vector_t>>(dstId, curBSLinks));
  } else {
    phyLinksAtBSs.at(dstId).push_back(lastHop);
  }
}

int evaluateSpaceDiversityAtNode(const int nodeId, const std::vector<Point_t>& nodes,
                                 std::vector<int>& maxSDNodeList,
                                 const std::vector<std::vector<int>>& nodeNeighborList,
                                 const SystemParameters& parameters) {
  int spaceDiversity = 0;
  double isoAngle = parameters.antennaIsoSpan_phi;
  std::vector<int> neighbors = nodeNeighborList[nodeId];  // the intended node's neighbors (indices)
  std::vector<std::vector<int>> graphIsoNeighbors(neighbors.size(), std::vector<int>());
  for (int i = 0; i< neighbors.size()-1; i++) {
    Point_t p1 = nodes[neighbors[i]];
    Vector_t v1(nodes[nodeId], p1);
    for (int j = i + 1; j < neighbors.size(); j++) {
      Point_t p2 = nodes[neighbors[j]];
      Vector_t v2(nodes[nodeId], p2);
      double a12 = acos(v1.dot(v2)/v1.mod()/v2.mod());
      if (a12 >= isoAngle) {
        graphIsoNeighbors[i].push_back(j);
        graphIsoNeighbors[j].push_back(i);
      }
    }
  }
  std::vector<int> P;
  for (int i = 0; i < neighbors.size(); ++i) {
    P.push_back(i);
  }
  std::vector<int> R;
  std::vector<int> X;
  std::vector<std::vector<int>> allMaximalCliques;
  int maxDegree = 0;
  BronKerboschPivoting(graphIsoNeighbors, R, P, X, allMaximalCliques, maxDegree);
  for (auto clique : allMaximalCliques) {
    if (clique.size() > spaceDiversity) {
      spaceDiversity = clique.size();
    }
  }
  return spaceDiversity;
}

/* Bron-Kerbosch algorithm with pivoting to list all maximal cliques in an arbitrary graph. */
void BronKerboschPivoting(const std::vector<std::vector<int>>& graph, const std::vector<int>& R, std::vector<int> P,
                          std::vector<int> X, std::vector<std::vector<int>>& allMaximalCliques, int& maxDegree){
  if (maxDegree >= 15) return;
    /* if P and X are both empty:  */
  if (P.empty() && X.empty()) {
    /* report R as a maximal clique */
    if (R.size() > maxDegree) {
        allMaximalCliques.push_back(R);
        maxDegree = R.size();
    }
  } else {
    if (R.size() + P.size() <= maxDegree) {
        return;
    }

    /* choose a pivot vertex u in P ⋃ X */
    std::sort(P.begin(), P.end());
    std::vector<int> PUX = P;
    PUX.insert(PUX.end(), X.begin(), X.end());
    int maxNumNeighborsInP = -1;
    int uSelected = -1;
    std::vector<int> uNeighbors;
    for (auto u : PUX) {
      int size1 = graph[u].size(); // neighbor of u in the graph (sorted)
      int size2 = P.size();
      std::vector<int> vIntersect(size1+size2);
      std::vector<int>::iterator it;
      it=std::set_intersection (graph[u].begin(), graph[u].end(), P.begin(), P.end(), vIntersect.begin());
      vIntersect.resize(it-vIntersect.begin());
      if ( (int) vIntersect.size() > maxNumNeighborsInP) {
        maxNumNeighborsInP = vIntersect.size();
        uSelected = u;
        uNeighbors = vIntersect;
      }
    }
    assert(uSelected >= 0);
    /* Remove each u's neighbor from P */
    std::vector<int> PMinusUNeighbors = P;
    for (auto i : uNeighbors) {
      std::vector<int>::iterator it = std::find(PMinusUNeighbors.begin(), PMinusUNeighbors.end(), i);
      PMinusUNeighbors.erase(it);
    }
    /* for each vertex v in P \ N(u): */
    for (auto v : PMinusUNeighbors) {
      std::vector<int> RUV = R;
      RUV.push_back(v);
      int size1 = graph[v].size();
      int size2 = P.size();
      std::sort(X.begin(), X.end());
      int size3 = X.size();
      std::vector<int> PIvNeighbors(size1+size2);
      std::vector<int> XIvNeighbors(size1+size3);
      std::vector<int>::iterator it = std::set_intersection(graph[v].begin(), graph[v].end(), P.begin(), P.end(), PIvNeighbors.begin());
      PIvNeighbors.resize(it - PIvNeighbors.begin());
      it = std::set_intersection(graph[v].begin(), graph[v].end(), X.begin(), X.end(), XIvNeighbors.begin());
      XIvNeighbors.resize(it - XIvNeighbors.begin());
      /* BronKerbosch2(R ⋃ {v}, P ⋂ N(v), X ⋂ N(v)) */
      BronKerboschPivoting(graph, RUV, PIvNeighbors, XIvNeighbors, allMaximalCliques, maxDegree);
      /* P := P \ {v} */
      it =  std::find(P.begin(), P.end(), v);
      P.erase(it);
      /* X := X ⋃ {v} */
      X.push_back(v);
    }
  }
}

bool checkTwoPhysicalLinksInterference(const std::vector<int>& phyLink1, const std::vector<int>& phyLink2,
                                       int& numRelays, const std::vector<Point_t>& nodes,
                                       const SystemParameters& parameters){
  /*
   * Check whether there are two nodes same to each other.
   * Since any physical link is from a src to a dst, and src != dst,
   * thus, it would only be possible that s1 == s2, s1 == d2, d1 == s2, or d1 == d2.
   */
  int s1 = phyLink1[0];  // phyLink stores the node id of src and dst
  int d1 = phyLink1[1];
  int s2 = phyLink2[0];
  int d2 = phyLink2[1];
  Vector_t s1d1(nodes[s1], nodes[d1]);
  Vector_t s2d2(nodes[s2], nodes[d2]);
  Vector_t s2d1(nodes[s2], nodes[d1]);
  Vector_t s1d2(nodes[s1], nodes[d2]);

  if (s1 == s2) {
    /* d1 != d2 */
    /* If angle d1sd2 >= isolation angle, there is no interference. */
    double a_d1sd2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
    if (a_d1sd2 >= parameters.antennaIsoSpan_phi)
      return false;
    else
      return true;
  }

  if (d1 == d2) {
    /* s1 != s2 */
    double a_s1ds2 = acos(s1d1.dot(s2d2)/s1d1.mod()/s2d2.mod());
    if (a_s1ds2 >= parameters.antennaIsoSpan_phi)
      return false;
    else
      return true;
  }

  if (s1 == d2) {
    double a_s2s1d1 = acos(-1 * s2d2.dot(s1d1)/s2d2.mod()/s1d1.mod());
    double a_s2d1s1 = acos(s2d1.dot(s1d1)/s2d1.mod()/s1d1.mod());
    double a_d1s2d2 = acos(s2d2.dot(s2d1)/s2d2.mod()/s2d1.mod());
    if (a_s2d1s1 < parameters.antennaBeamWidth_phi/2 || a_d1s2d2 < parameters.antennaBeamWidth_phi/2
        || a_s2s1d1 < parameters.antennaBeamWidth_phi)
      return true;
    else
      return false;
  }

  if (s2 == d1) {
    double a_s1s2d2 = acos(-1 * s2d2.dot(s1d1) / s2d2.mod() / s1d1.mod());
    double a_d2s1d1 = acos(s1d2.dot(s1d1) / s1d2.mod() / s1d1.mod());
    double a_s1d2s2 = acos(s1d2.dot(s2d2) / s1d2.mod() / s2d2.mod());
    if (a_d2s1d1 < parameters.antennaBeamWidth_phi / 2 || a_s1d2s2 < parameters.antennaBeamWidth_phi / 2
        || a_s1s2d2 < parameters.antennaBeamWidth_phi)
      return true;
    else
      return false;
  }

  double a_s1d1s2 = acos(s1d1.dot(s2d1)/s1d1.mod()/s2d1.mod());
  double a_d1s2d2 = acos(s2d1.dot(s2d2)/s2d1.mod()/s2d2.mod());
  double a_s1d2s2 = acos(s1d2.dot(s2d2)/s1d2.mod()/s2d2.mod());
  double a_d1s1d2 = acos(s1d1.dot(s1d2)/s1d1.mod()/s1d2.mod());
  if (a_s1d1s2 < parameters.antennaBeamWidth_phi/2 || a_d1s2d2 < parameters.antennaBeamWidth_phi/2
      || a_s1d2s2 < parameters.antennaBeamWidth_phi/2 || a_d1s1d2 < parameters.antennaBeamWidth_phi/2) {
    return true;
  } else {
    return false;
  }
}

void collectMutualInterferenceInfo(std::vector<std::vector<int>>& mutualInterferenceIndicator,
                                   const std::vector<std::vector<int>>& phyLinkSet, int& numRelays,
                                   const std::vector<Point_t>& nodes, const SystemParameters& parameters,
                                   std::string& dataMutualInterference, bool& write){
    std::ofstream outFile;
    outFile.open(dataMutualInterference, std::ios_base::app);
    if (!outFile.is_open()) {
        cout << "(E) Fail to open the file where mutual interference info should be stored. " << endl;
    } else if (write) {
        cout << "(O) Ready to write the mutual interference info into file." << endl;
    }
    int numPhyLinks = phyLinkSet.size();
    for (int i = 0; i < numPhyLinks; ++i) {
        for (int j = 0; j < numPhyLinks; ++j) {
            if (i == j) {
                mutualInterferenceIndicator[i][j] = 1;  // A link interfered with itself
            } else {
                bool result = checkTwoPhysicalLinksInterference(phyLinkSet[i], phyLinkSet[j], numRelays, nodes, parameters);
                if (result) {
                    mutualInterferenceIndicator[i][j] = 1;
                } else {
                    mutualInterferenceIndicator[i][j] = 0;
                }
            }
            if (write) {
                outFile << mutualInterferenceIndicator[i][j] << "\t";
            }
        }
        if (write) {
            outFile << "\n";
        }
    }
    outFile.close();
}