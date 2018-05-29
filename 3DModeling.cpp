//
// Created by qiang on 4/16/18.
//

#include "3DModeling.h"


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
 * Read in building information from data files.
 */
std::vector<Building_t> getBuildingInfoFromFile(std::string dataBuildings, std::string dataBuildingVertices, SystemParameters& parameters) {
  std::ifstream fileIn(dataBuildings);
  std::ofstream fileOut;
  std::string str;
  std::vector<std::string> data;
  while (std::getline(fileIn, str, '\t'))
  {
    if (str.find('\n') != std::string::npos){
      data.push_back(str.substr(0, str.find('\n')));
      data.push_back(str.substr(str.find('\n')+1));
    } else {
      data.push_back(str);
    }
  }
//  cout<< data.size() << endl;
  // Create each building objects and store them.
  std::vector<Building_t> buildings;
  fileOut.open(dataBuildingVertices, std::ios_base::app);
  if (fileOut.is_open()){
    cout << "Ready to write building vertices information to file." << endl;
  } else {
    cout << "Fail to open the file where building vertices information should be stored." << endl;
  }
  for (int i = 0; i < data.size(); i = i+7){
    double center[2] {std::stod(data.at(i)), std::stod(data.at(i+1))};
    double length_m = std::stod(data.at(i+2));
    double width_m = std::stod(data.at(i+3));
    double topHeight_m = std::stod(data.at(i+4));
    double baseLevel_m = std::stod(data.at(i+5));
    double lwhbg[] {length_m, width_m, topHeight_m, baseLevel_m + parameters.minHeightForRelay_m, parameters.groundLevel_m};
    double orientation_rad = (std::stod(data.at(i+6)))/180.0*M_PI;
    /* Generate each building object. */
    Building_t newBuilding(center, lwhbg, orientation_rad, parameters.maxHeightForRelay_m, parameters.densityRelayOnBuilding, parameters.randomSeed);
    buildings.push_back(newBuilding);
    fileOut << (buildings.at(buildings.size()-1).toStringData()+"\n");
//    cout << buildings.at(buildings.size()-1)->toStringData() << endl;
  }

  fileOut.close();
  return buildings;
}

std::vector<Point_t> generateCandidateBaseStations(std::vector<Building_t>& buildingSet, std::vector<Point_t>& roofTopRelays, SystemParameters& parameters){
  /* Set up random generator. */
  // Seed with a real random value, if available
  std::random_device r;
  std::default_random_engine e(r());
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
      Point_t newBS(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId++) % 4), pro));
      poolBS.push_back(newBS);
      Point_t newRelay0(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(newRelay0);
      Point_t newRelay1(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(newRelay1);
      Point_t newRelay2(proportionalPoint(baseVertices.at(caseId % 4), topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(newRelay2);
      // System.out.println("No. "+ countBS+ "\t BS candidate position at " + candiBS);
      // StdDraw.point(candiBS.x, candiBS.y);
    }
  }
  cout << "(1) There are " + to_string(poolBS.size()) + " candidate base stations being generated." << endl;
  cout << "(2) There are " + to_string(roofTopRelays.size()) + " candidate roof top relays being generated." << endl;

  return poolBS;
}

void selectBaseStationPerGrid(std::vector<Point_t>& bsSet, SystemParameters& parameters){
  /* Number of grids. */
  auto numGridAlongX = (unsigned int) ((parameters.areaXRange_m[1]-parameters.areaXRange_m[0])/parameters.gridSize_m);
  auto numGridAlongY = (unsigned int) ((parameters.areaYRange_m[1]-parameters.areaYRange_m[0])/parameters.gridSize_m);
  /* Initialize the Map for indicating whether there is a BS in the grid. */
  std::vector<bool> gridHasBS;
  for (int i = 0; i < numGridAlongX; i++){
    for (int j = 0; j < numGridAlongY; j++){
      gridHasBS.push_back(false);
    }
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
      gridHasBS.at(gridIndexX*numGridAlongY + gridIndexY) = true;
//      StdDraw.point(currentBS.x, currentBS.y);
      i++;
    }
  }
  cout << "(3) There are " + to_string(bsSet.size()) + " candidate base stations being selected." << endl;
  /* Write the selected base stations to file. */
  ofstream outFile;
  outFile.open("../Data/Base_Stations/bsSet_"+parameters.simStartTime+".txt", std::ios_base::app);
  if (!outFile.is_open()) {
    cerr << "Error!!!The file to store base stations is not open!!" << endl;
    exit(errno);
  }
  for(auto bs : bsSet) {
    outFile << bs.toStringData() << endl;
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

std::vector<Point_t> collectAllRelays(const std::vector<Building_t>& buildings){
  std::vector<Point_t> allRelays;
  for (const Building_t& bldg : buildings){
    std::vector<Point_t> curRelays = bldg.getRelays();
    allRelays.insert(allRelays.begin(), curRelays.begin(), curRelays.end());
  }
  cout << "(4) There are " + to_string(allRelays.size()) + " candidate relays on the surfaces of buildings." << endl;
  return allRelays;
}

std::vector<std::vector<int>> exploreConnectivity(const std::vector<Point_t>& nodes, const std::vector<Building_t>& buildings, const std::string& fileRelayNeighbors){
  std::vector<std::vector<int>> neighborList;
  std::ofstream fileOut;
  fileOut.open(fileRelayNeighbors, std::ios_base::app);
  if (fileOut.is_open()){
    cout << "Ready to write neighbors information to file." << endl;
  } else {
    cout << "Fail to open the file where neighbors information should be stored." << endl;
  }
  for (unsigned int i = 0; i < nodes.size(); i++) {
    std::vector<int> curRelayNeighbors = searchNonBlockLink(buildings, (nodes.at(i)), nodes);
    neighborList.push_back(curRelayNeighbors);
    std::string outputToFile = "";
    for (unsigned int j = 0; j < curRelayNeighbors.size(); ++j){
      outputToFile += std::to_string(curRelayNeighbors.at(j)) + "\t";
    }
    fileOut << outputToFile + "\n";
    cout << "The No.\t" + std::to_string(i) + "\tnode has\t" + std::to_string(neighborList.at(i).size()) + "\tnon-block neighbors." << endl;
  }
  fileOut.close();
  return neighborList;
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
  std::vector<int>* data = new std::vector<int>();
  while (std::getline(fileIn, str, '\t'))
  {
    if (str.find('\n') != std::string::npos){
//      data.push_back(str.substr(0, str.find('\n')));
      relayNeighborList.push_back(*data);
      data = new std::vector<int>();
      std::string temp = str.substr(str.find('\n')+1);
      if (temp != "") {
        data->push_back(std::stoi(str.substr(str.find('\n')+1)));
      }
    } else {
      data->push_back(std::stoi(str));
    }
  }

  cout << "Connectivity information loaded!" << endl;
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
  for (int i=0; i<oldNodes.size(); i++){
    Line_t sd(newNode, oldNodes.at(i));
    bool blockTest = blockageTest(buildings, sd);
    if (!blockTest){
      nonBlockNodes.push_back(i);
      currentList.at(i).push_back(oldNodes.size());
    }
  }
  currentList.push_back(nonBlockNodes);
  return currentList;
}

std::vector<std::vector<int>> findPathDecodeForward(const std::vector<std::vector<int>>& nodeNeighborList,
                                                    const std::vector<Point_t>& nodes, int addHop,
                                                    SystemParameters& parameters){
  /*
   * Get source and destination.
   */
  int numNodes = nodes.size();
  int srcIndex = numNodes - 2;
  int dstIndex = numNodes - 1;
  Point_t src = nodes[srcIndex];
  Point_t dst = nodes[dstIndex];

  /*
   * Initialization: allPaths store: shortest hop optimal path; plus 1 optimal path; ...
   * in total there are "addHop + 1" paths, but one more path will be added if shortest path is LoS path
   */
  std::vector<std::vector<int>> allPaths; // to store all final optimal paths with different number of hops
  parameters.lowerBound_Gbps = 0; // Reset the lower bound of the path throughput as 0.

  /*
   * Find a path with the minimum number of hops. Using Dijkstra algorithm with type "hop"
   */
  std::string pathFileDijkstra = "../Data/Paths/Dijkstra.txt";
  std::vector<int> pathDijkstraHop = Dijkstra(nodeNeighborList, nodes, pathFileDijkstra, "hop");
  if (pathDijkstraHop.empty()){
    cout << "Warning!!! There is no path from current source to destination!!\n";
    return allPaths;  // the return value is also null
  }
  cout << "The minimum number of hops path has " + std::to_string(pathDijkstraHop.size()-1) + " hops." << endl;

  /*
   * Calculate the current end-to-end throughput.
   */
  double pathThroughput_Gbps = 40.0;  // Even the shortest link cannot exceed 35.6 Gbps throughput due to upper limit on SNR
  int minHop = pathDijkstraHop.size() - 1;
  if (minHop == 1) {
    // Special case: LoS path exists
    allPaths.push_back(pathDijkstraHop);  // the LoS path is added to allPaths.
    minHop = 2; // In the later search, directly start to search path with 2 hops.
    /*
     * Do not update the parameters.lowerBound_Gbps, because the los path usually has very high throughput due to no primary constraint.
     */
  } else {
    // At least 2 hops in the path with smallest number of hops
    // Update the current path throughput.
    for (int i = 0; i < minHop - 1; ++i) {
      // current hop is i to i+1
      // next hop is i+1 to i+2
      double linkLengthCurrent_m = nodes[pathDijkstraHop.at(i)].distanceTo(nodes[pathDijkstraHop.at(i+1)]);
      double linkLengthNext_m = nodes[pathDijkstraHop.at(i+1)].distanceTo(nodes[pathDijkstraHop.at(i+2)]);
      double capacityCurrent_Gbps = calculateLinkCapacity_Gbps(linkLengthCurrent_m, parameters);
      double capacityNext_Gbps = calculateLinkCapacity_Gbps(linkLengthNext_m, parameters);
      double throughputCurrent_Gbps = capacityCurrent_Gbps * capacityNext_Gbps / (capacityCurrent_Gbps + capacityNext_Gbps);
      // A path's throughput is the smallest throughput of a pair of consecutive links
      if (throughputCurrent_Gbps < pathThroughput_Gbps){
        pathThroughput_Gbps = throughputCurrent_Gbps;
      }
    }
    parameters.lowerBound_Gbps = pathThroughput_Gbps; // As long as a path is found, update the lowerBound_Gbps
  }

  if (pathThroughput_Gbps >= 20.0 && pathThroughput_Gbps != 40.0) {
    cerr << "Warning!!! There is something wrong with the calculation on capacity.\n";
    exit(errno);
  }

  /* Iterate the path find process "addHop" times to obtain the path with different lengths. */
  std::chrono::microseconds curMs = std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()
  );
  std::string curTime = std::to_string(curMs.count()/1000);
  std::string pathFileDF = "../Data/Paths/" + curTime + "DF.txt";
  std::ofstream fileOutDF;
  fileOutDF.open(pathFileDF, std::ios_base::app);
  // do not allow the number of hops to exceed 10 hops.
  for (int i = 0; i <= addHop && i <= parameters.hopLimit - minHop; ++i) {
    int maxHop = minHop + i;
    cout << "============ " + std::to_string(maxHop) + " hop case ============\n";
    // Find the path with no more than maxHop hops which has the optimal throughput.
    std::vector<int> path = findPathDecodeForwardMaxHop(nodeNeighborList, nodes, maxHop, parameters);
    if (path.empty()){
      cout << "Warning!!! There is no path with no more than " + std::to_string(maxHop) + " hops\n";
      addHop++;
    } else {
      allPaths.push_back(path);   // allPaths may contain null path.
      for (int j : path){
        fileOutDF << nodes.at(j).toStringData() << "\n";
      }
    }
  }
  fileOutDF.close();

  return allPaths;
}

std::vector<int> findPathDecodeForwardMaxHop(const std::vector<std::vector<int>>& nodeNeighborList,
                                             const std::vector<Point_t>& nodes, int maxHop,
                                             SystemParameters& parameters){
  /*
   * Get source and destination.
  */
  int numNodes = nodes.size();
  int srcIndex = numNodes - 2;
  int dstIndex = numNodes - 1;
  Point_t src = nodes[srcIndex];
  Point_t dst = nodes[dstIndex];

  /*
   * Initialization: path is used to store the final result.
   */
  std::vector<int> pathMaxHop;
  double preHopCap_Gbps = 40.0; // At the very beginning, there is no previous hop, so the preHopCap_Gbps is set as a value which cannot be achieved.
  std::vector<std::vector<int>> pathList = findNextHopNode(nodeNeighborList, nodes, maxHop, parameters, srcIndex, 0, 40.0, 40.0);
  if (!pathList.empty()){
    int maxThroughtput = 0;
    int indexMax = 0;
    for (int i = 0; i < pathList.size(); ++i) {
      /* When more than one hop is needed! */
      if (maxHop > 1 && pathList.at(i).size() == 2) continue;
      if (pathList.at(i).at(0) > maxThroughtput){
        // System.out.println("Find the optimal path under the maximum hop limit.");
        maxThroughtput = pathList.at(i).at(0);
        indexMax = i;
      }
    }
    if (pathList.size() > 1 && maxThroughtput == 0){
      cerr << "Error!!! No path has above 0 throughput!!\n";
      exit(errno);
    } else {
      pathMaxHop.push_back(srcIndex);
      for (int j = pathList.at(indexMax).size()-1; j >= 1; --j){
        pathMaxHop.push_back(pathList.at(indexMax).at(j));
      }
    }
    if (pathMaxHop.empty()){
      cerr << "Error!!! Do not find the best path.\n";
      return pathMaxHop;
    }
  }

  return pathMaxHop;
}


void searchPathDecodeForwardMaxHop(Path_t& paths, const std::vector<Point_t>& nodes,
                                   const std::vector<std::vector<int>>& nodeNeighborList,
                                   const int& relayNum, SystemParameters& parameters){
  /*
   * Get source and destination of this path searching process.
   */
  Point_t src = nodes[paths.getSrcId()];
  Point_t dst = nodes[paths.getDstId()];
  /* Initialize a path with source node id in it. */
  std::vector<int> curPath;
  curPath.push_back(paths.getSrcId());

  searchNextHopNode(paths, curPath, nodes, nodeNeighborList, relayNum, 0, paths.getMaxHopNum(), 40, 40, parameters);
}


void searchNextHopNode(Path_t& paths, const std::vector<int>& curPath, const std::vector<Point_t>& nodes,
                       const std::vector<std::vector<int>>& nodeNeighborList, const int& relayNum,
                       const int& preHopNum, const int& maxHopNum, const double& preHopCap,
                       const double& curPathThroughput, SystemParameters& parameters) {
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
                            updatePathThroughput, parameters);
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

std::vector<std::vector<int>> findNextHopNode(const std::vector<std::vector<int>>& nodeNeighborList,
                                              const std::vector<Point_t>& nodes, int maxHop, SystemParameters& parameters,
                                              int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput){
  // int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput
  double prePathThroughput = pathThroughput;
  std::vector<std::vector<int>> validPaths;
  /* The destination node is always the last node in the "nodes" vector. */
  Point_t dst = nodes[nodes.size() - 1];
  /* Update the current hop number. The searching process starts from hop 0 at src. */
  int curHopNum = preHopNum + 1;
  /*
   * If current hop exceeds the maximum hop number allowed, return a null path.
   * When the returned path vector is empty, it means no path has been found.
   */
  if (curHopNum > maxHop) {
    return validPaths;
  } else {
    /*
     * Here, the current hop number is below hop number upper bound.
     * It also implies that the node previously selected is not the destination.
     * The following assertion make sure that the searching ends at the destination.
     */
    assert(preNodeIndex != (nodes.size() - 1));
    /*
     * Get candidate nodes of this hop, which are the neighboring nodes of the previous node.
     */
    Point_t preNode = nodes[preNodeIndex];  // the previous node
    std::vector<int> candidates = nodeNeighborList[preNodeIndex]; // The indices of all neighbors of the previous node
    /*
     * Iterates each candidate node for the next node to be inserted into the path.
     */
    for (int i = 0; i < candidates.size(); ++i){
      Point_t curNode = nodes[candidates[i]]; // "candidates" only stores indices of neighboring nodes.
      double linkLength_m = preNode.distanceTo(curNode);
      double distToDst_m = curNode.distanceTo(dst);
      /* Test the link length, which should be smaller than the threshold. */
      if (linkLength_m > parameters.phyLinkDistMax_m) continue;
      /* Control the expansion of the route. */
      if (candidates.at(i) != (nodes.size() - 1) && distToDst_m > parameters.phyLinkDistMax_m * (maxHop - curHopNum)) continue;
      /* Calculate the current throughput of the consecutive link pair, when the candidate neighbor is selected. */
      double curHopCap = calculateLinkCapacity_Gbps(linkLength_m, parameters);
      double curThroughput = preHopCap * curHopCap / (preHopCap + curHopCap);
      /* The throughput of the consecutive link pair should be larger than the lower bound. */
      if (curThroughput < parameters.lowerBound_Gbps) continue;
      if (pathThroughput < parameters.lowerBound_Gbps) continue;
      /* Update the current path throughput when this node is selected. */
      double curPathThroughput = prePathThroughput;
      if (curThroughput < curPathThroughput) {
        curPathThroughput = curThroughput;
      }
      if (candidates.at(i) == (nodes.size() -1)){
        /* The currently selected node is the destination node. */
        std::vector<int> validSinglePath; // define a vector to store the found single path.
        /*
         * Push back the path throughput into the path vector as the first element.
         * If the path is LoS, the path throughput should be curHopCap.
         * Otherwise, the path throughput should be curPathThroughput.
         */
        if (curHopNum == 1) {
          /* The path is a line of sight single hop path. */
          validSinglePath.push_back((int) (curHopCap * 10000));
        } else {
          validSinglePath.push_back((int) (curPathThroughput * 10000));
          /* Only update the lower bound of the path throughput when there is no LoS path. */
          parameters.lowerBound_Gbps = curPathThroughput;
          cout << "The lower bound of the throughput in this search becomes " + std::to_string(curPathThroughput) + " Gbps.\n";
        }
        /* Push back the current node which is also the destination node into the path vector. */
        validSinglePath.push_back(candidates[i]);
        /* Push back the current path to the path list. */
        validPaths.push_back(validSinglePath);
      } else {
        /*
         * Current selected node is not the destination node. The searching process continues.
         */
        std::vector<std::vector<int>> pathList = findNextHopNode(nodeNeighborList, nodes, maxHop, parameters, candidates.at(i), curHopNum, curHopCap, curPathThroughput);
        if (pathList.empty()) {
          // do nothing, because no path has been found.
        } else {
          for (auto path0 : pathList){
            /* Only return the paths which do not have duplicated nodes. */
            if (std::find(path0.begin(), path0.end(), candidates[i]) == path0.end() && candidates[i] != nodes.size()-2){
              path0.push_back(candidates.at(i));
              validPaths.push_back(path0);
            }
          }
        }
      }
    }
    // Updated all paths found in validPaths
    return validPaths;
  }
}

/*
 * Dijkstra algorithm.
 */
std::vector<int> Dijkstra(const std::vector<std::vector<int>>& neighborList, const std::vector<Point_t>& nodes,
                          std::string pathFile, std::string type){
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
  distToSrc[nodes.size() - 2] = 0;
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
      cout << "There are " + std::to_string(unvisitedSet.size()) + " nodes unvisited, but they are not reachable.\n";
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
        cout << "Warning: The distance 'alt' is larger than 1.0E10.\n";
        break;
      }
    }
  }

  std::vector<int> path;
  int currentIndex = nodes.size() - 1; // destination node
  while (currentIndex != (nodes.size()-2)){
    path.push_back(currentIndex);
    if (prevIndex[currentIndex] == -1){
      cout << "There is no path to the destination, because the current node does not have previous hop.\n";
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

bool checkTwoPathsInterference(const std::vector<int>& path1, const std::vector<int>& path2,
                               const std::vector<Point_t>& sd1, const std::vector<Point_t>& sd2,
                               const std::vector<std::vector<int>>& nodeNeighborList,
                               const std::vector<Building_t>& buildings,
                               const std::vector<Point_t>& nodes, const SystemParameters& parameters) {
  int hop1 = path1.size() - 1;
  int hop2 = path2.size() - 1;
  double halfBeam = parameters.antennaBeamWidth_phi/2.0;

//  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
//  std::vector<std::vector<int>> nodeNeighborList = addNodeToConnectivityList(relayNeighborList, sd1[0], nodes, buildings);
//  std::vector<Point_t> allNodes = nodes;
//  allNodes.push_back(sd1[0]);
//  nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, sd1[1], allNodes, buildings);
//  allNodes.push_back(sd1[1]);
//  if (sd2[0].sameAs(sd1[0])){
//    int curIdx = allNodes.size();
//    int refIdx = allNodes.size()-2;
//    std::vector<int> curNeighbors = nodeNeighborList[refIdx];
//    nodeNeighborList.push_back(curNeighbors);
//    for (auto idx: curNeighbors) {
//      nodeNeighborList[idx].push_back(curIdx);
//    }
//  } else if (sd2[0].sameAs(sd1[1])){
//    int curIdx = allNodes.size();
//    int refIdx = allNodes.size()-1;
//    std::vector<int> curNeighbors = nodeNeighborList[refIdx];
//    nodeNeighborList.push_back(curNeighbors);
//    for (auto idx: curNeighbors) {
//      nodeNeighborList[idx].push_back(curIdx);
//    }
//  } else {
//    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, sd2[0], allNodes, buildings);
//  }
//  allNodes.push_back(sd2[0]);
//  if (sd2[1].sameAs(sd1[0])){
//    int curIdx = allNodes.size();
//    int refIdx = allNodes.size()-3;
//    std::vector<int> curNeighbors = nodeNeighborList[refIdx];
//    nodeNeighborList.push_back(curNeighbors);
//    for (auto idx: curNeighbors) {
//      nodeNeighborList[idx].push_back(curIdx);
//    }
//  } else if (sd2[1].sameAs(sd1[1])) {
//    int curIdx = allNodes.size();
//    int refIdx = allNodes.size()-2;
//    std::vector<int> curNeighbors = nodeNeighborList[refIdx];
//    nodeNeighborList.push_back(curNeighbors);
//    for (auto idx: curNeighbors) {
//      nodeNeighborList[idx].push_back(curIdx);
//    }
//  } else {
//    nodeNeighborList = addNodeToConnectivityList(nodeNeighborList, sd2[1], allNodes, buildings);
//  }
//  allNodes.push_back(sd2[1]);
//  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
//  std::cout << "It took " << time_span.count() << " seconds to add end points to neighbor list." << endl;

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

//      /*
//       * If the two physical links share any of the end point:
//       * s1 = s2, s1 = d2, d1 = s2, d1 = d2
//       */
//      if (s1.sameAs(s2)) {
//        if (d1.sameAs(d2)) return true; // two physical links are the same, must interfere with each other.
//        // d1 != d2, and as s1!=d1, s2!=d2 ==> s1!=d2, s2!=d1
//
//
//      }
//
//      if (s1.sameAs(s2) || s1.sameAs(d2) || d1.sameAs(s2) || d1.sameAs(d2)) {
//        if ()
//      }
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
            if (a_d1s1d2 <= 0.0001 && a_s1d2s2 < M_PI/6.0) {
              return true;
            }
            if (a_s1d2s2 <= 0.0001 && a_d1s1d2 < M_PI/6.0) {
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
            if (a_s1d1s2 <= 0.0001 && a_d1s2d2 < M_PI/6.0) {
              return true;
            }
            if (a_d1s2d2 <= 0.0001 && a_s1d1s2 < M_PI/6.0) {
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
            if (a_s1d1d2 <= 0.0001 && a_s2d2d1 < M_PI/6.0) {
              return true;
            }
            if (a_s2d2d1 <= 0.0001 && a_s1d1d2 < M_PI/6.0) {
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
            if (a_s2s1d1 <= 0.0001 && a_d2s2s1 < M_PI/6.0) {
              return true;
            }
            if (a_d2s2s1 <= 0.0001 && a_s2s1d1 < M_PI/6.0) {
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