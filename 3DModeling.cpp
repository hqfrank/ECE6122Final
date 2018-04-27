//
// Created by qiang on 4/16/18.
//

#include "3DModeling.h"

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

std::vector<Point_t> generateBaseStationPairs(const std::vector<Point_t>& bsSet, SystemParameters& parameters){
  /* Set up random generator. */
  // Seed with a real random value, if available
  std::random_device r;
  std::default_random_engine e(r());
  std::uniform_int_distribution<int> uniform_dist(0, bsSet.size()-1);
  /* Initialize the (source,destination) pair */
  std::vector<Point_t> sdPair((unsigned) parameters.numBSPairs * 2);
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
      sdPair.at((unsigned) countPair * 2) = srcRnd;
      sdPair.at((unsigned) countPair * 2 + 1) = dstRnd;
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
      if (maxHop > 1 && pathList.at(i).size() == 2) continue;
      if (pathList.at(i).at(0) > maxThroughtput){
        // System.out.println("Find the optimal path under the maximum hop limit.");
        maxThroughtput = pathList.at(i).at(0);
        indexMax = i;
      }
    }
    if (maxThroughtput == 0){
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

std::vector<std::vector<int>> findNextHopNode(const std::vector<std::vector<int>>& nodeNeighborList,
                                              const std::vector<Point_t>& nodes, int maxHop, SystemParameters& parameters,
                                              int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput){
  // int preNodeIndex, int preHopNum, double preHopCap, double pathThroughput
  double prePathThroughput = pathThroughput;
  std::vector<std::vector<int>> validPaths;
  Point_t dst = nodes[nodes.size() - 1];  // get the destination node
  int curHopNum = preHopNum + 1;  // the hop number of this hop
  if (curHopNum > maxHop) { // if current hop exceeds the maximum hop number allowed, return a null path.
    return validPaths;
  } else {
    // hop number is OK, get candidate nodes of this hop
    Point_t preNode = nodes[preNodeIndex];  // the previous node
    std::vector<int> candidates = nodeNeighborList.at(preNodeIndex); // The indices of all neighbors of the previous node
    for (int i = 0; i < candidates.size(); ++i){
      // iterate each candidate node
      Point_t curNode = nodes[candidates.at(i)]; // read the i-th candidate node for this hop
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
      double curPathThroughput = pathThroughput;
      if (curThroughput < curPathThroughput) {
        curPathThroughput = curThroughput;
      }
      if (candidates.at(i) == (nodes.size() -1) && curHopNum > 1){
        // Current selected node is destination and the path throughput is larger than lowerBound_Gbps
        std::vector<int> validSinglePath;
        validSinglePath.push_back((int) (curPathThroughput * 10000));
        validSinglePath.push_back(candidates.at(i));
        parameters.lowerBound_Gbps = curPathThroughput;
        cout << "The lower bound of the throughput in this search becomes " + std::to_string(curPathThroughput) + " Gbps.\n";
        validPaths.push_back(validSinglePath);
      } else {
        // Current selected node is not the destination node.
        std::vector<std::vector<int>> pathList = findNextHopNode(nodeNeighborList, nodes, maxHop, parameters, candidates.at(i), curHopNum, curHopCap, curPathThroughput);
        if (pathList.empty()) {
          // do nothing
        } else {
          for (int j = 0; j < pathList.size(); ++j){
            pathList.at(j).push_back(candidates.at(i));
            validPaths.push_back(pathList.at(j));
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