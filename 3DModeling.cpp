//
// Created by qiang on 4/16/18.
//

#include "3DModeling.h"

/*
 * Calculate the point at the position p between p1 and p2, such that |p1p|= pro*|p1p2|
 */
Point_t* proportionalPoint(Point_t& p1, Point_t& p2, double pro){
  assert(pro >= 0);
  Vector_t* v12 = new Vector_t(p1, p2);
  Vector_t* v = new Vector_t(*v12, pro);
  return p1.destinationTo(*v);
}

/*
 * Dot product of two vectors
 */
double dot(Vector_t& v1, Vector_t& v2){
  return (v1.getX() * v2.getX()) + (v1.getY() * v2.getY()) + (v1.getZ() * v2.getZ());
}

Vector_t* cross(Vector_t& v1, Vector_t& v2){
  return new Vector_t(v1.getY() * v2.getZ() - v1.getZ() * v2.getY(), v1.getZ() * v2.getX() - v1.getX() * v2.getZ(), v1.getX() * v2.getY() - v1.getY() * v2.getX());
}

Vector_t* normalize(Vector_t& v){
  return v.divide(v.mod());
}

/*
 * Read in building information from data files.
 */
std::vector<Building_t*> getBuildingInfoFromFile(std::string dataBuildings, std::string dataBuildingVertices, SystemParameters parameters) {
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
  std::vector<Building_t*> buildings;
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
    buildings.push_back(new Building_t(center, lwhbg, orientation_rad, parameters.maxHeightForRelay_m, parameters.densityRelayOnBuilding, parameters.randomSeed));
    fileOut << (buildings.at(buildings.size()-1)->toStringData()+"\n");
//    cout << buildings.at(buildings.size()-1)->toStringData() << endl;
  }

  fileOut.close();
  return buildings;
}

std::vector<Point_t*> generateCandidateBaseStations(std::vector<Building_t*> buildingSet, std::vector<Point_t*>& roofTopRelays, SystemParameters parameters){
  /* Set up random generator. */
  // Seed with a real random value, if available
  std::random_device r;
  std::default_random_engine e(r());
  std::uniform_int_distribution<int> uniform_dist(0, 999);

  /* Initialize variables for storing BSs' locations. */
  std::vector<Point_t*> poolBS;
  int numBuildings = buildingSet.size();
  int countBS = 0;
  /* Pick one of the vertices of a building satisfying the height requirement as a candidate BS. */
  for(unsigned int i=0; i<numBuildings; i++){
    /* Test the building's height. */
    double minHeightBuilding = buildingSet.at(i)->getHeightBase() - parameters.minHeightForRelay_m + parameters.minHeightForBS_m;
    double maxHeightBuilding = buildingSet.at(i)->getHeightBase() - parameters.minHeightForRelay_m + parameters.maxHeightForBS_m;
    double randomDouble = uniform_dist(e) / 1000.0;
    auto caseId = (unsigned int) floor(randomDouble * 4.0);
    if (buildingSet.at(i)->getHeight() >= minHeightBuilding && buildingSet.at(i)->getHeight() <= maxHeightBuilding) {
      // Read all 4 top vertices of building i
      std::vector<Point_t *> topVertices = buildingSet.at(i)->getVts();
      poolBS.push_back(topVertices.at((caseId++) % 4));
      // This building has a candidate BS location.
      buildingSet.at(i)->setHasBS(true);
      // Add the other 3 top corners as the roof top relay locations.
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
      roofTopRelays.push_back(topVertices.at((caseId++) % 4));
    }
    if (buildingSet.at(i)->getHeight() > maxHeightBuilding){
      // Calculate the effective building height.
      double buildingHeight = buildingSet.at(i)->getHeight() - buildingSet.at(i)->getHeightBase();
      double randomHeight = uniform_dist(e) / 1000.0 * (parameters.maxHeightForBS_m-parameters.minHeightForBS_m) + parameters.minHeightForBS_m;
      double pro = randomHeight / buildingHeight;
      // Read all 4 top vertices of building i
      std::vector<Point_t *> topVertices = buildingSet.at(i)->getVts();
      std::vector<Point_t *> baseVertices = buildingSet.at(i)->getVbs();
      poolBS.push_back(proportionalPoint(*baseVertices.at(caseId % 4), *topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(proportionalPoint(*baseVertices.at(caseId % 4), *topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(proportionalPoint(*baseVertices.at(caseId % 4), *topVertices.at((caseId++) % 4), pro));
      roofTopRelays.push_back(proportionalPoint(*baseVertices.at(caseId % 4), *topVertices.at((caseId++) % 4), pro));
      // System.out.println("No. "+ countBS+ "\t BS candidate position at " + candiBS);
      // StdDraw.point(candiBS.x, candiBS.y);
    }
  }
  cout << "(1) There are " + to_string(poolBS.size()) + " candidate base stations being generated." << endl;
  cout << "(2) There are " + to_string(roofTopRelays.size()) + " candidate roof top relays being generated." << endl;

  return poolBS;
}

void selectBaseStationPerGrid(std::vector<Point_t*>& bsSet, SystemParameters parameters){
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
    Point_t currentBS = *(bsSet.at(i));
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

void selectRelayPerGrid(std::vector<Point_t*>& relays, SystemParameters parameters){
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
    Point_t currentRelay = *(relays.at(i));
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

std::vector<Point_t*> collectAllRelays(std::vector<Building_t*> buildings){
  std::vector<Point_t*> allRelays;
  for (auto bldg : buildings){
    std::vector<Point_t*> curRelays = bldg->getRelays();
    allRelays.insert(allRelays.begin(), curRelays.begin(), curRelays.end());
  }
  cout << "(4) There are " + to_string(allRelays.size()) + " candidate relays on the surfaces of buildings." << endl;
  return allRelays;
}

std::vector<std::vector<int>> exploreConnectivity(std::vector<Point_t*>& nodes, std::vector<Building_t*>& buildings, const std::string& fileRelayNeighbors){
  std::vector<std::vector<int>> neighborList;
  std::ofstream fileOut;
  fileOut.open(fileRelayNeighbors, std::ios_base::app);
  if (fileOut.is_open()){
    cout << "Ready to write neighbors information to file." << endl;
  } else {
    cout << "Fail to open the file where neighbors information should be stored." << endl;
  }
  for (unsigned int i = 0; i < nodes.size(); i++) {
    std::vector<int> curRelayNeighbors = searchNonBlockLink(buildings, *(nodes.at(i)), nodes);
    neighborList.push_back(curRelayNeighbors);
    std::string outputToFile = "";
    for (unsigned int j = 0; j < curRelayNeighbors.size(); ++j){
      outputToFile += std::to_string(curRelayNeighbors.at(j)) + "\t";
    }
    fileOut << outputToFile + "\n";
    cout << "The No.\t" + std::to_string(i) + "\tnode has\t" + std::to_string(neighborList.at(i).size()) + "\tnon-block neighbors." << endl;
  }
  return neighborList;
}


std::vector<int> searchNonBlockLink(std::vector<Building_t*>& buildings, Point_t& s, std::vector<Point_t*>& nodes){
  std::vector<int> nonBlockNodes;
  for(unsigned int i = 0; i < nodes.size(); i++){
    // for each node in the topology, determine whether there is a non-blocked path between s and nodes[i]
    double dist = nodes.at(i)->distanceTo(s); // the distance between s and nodes[i]
    if(dist >= 1){
      // the link length is valid; otherwise that relays[i] will not be added to the list
      Line_t* sd = new Line_t(s, *(nodes.at(i)));
      bool blockTest = blockageTest(buildings, *sd);
      if(!blockTest){
        // there is no blockage between s and relays[i]
        nonBlockNodes.push_back(i); // add the index of node to the list
      }
    }
  }
  return nonBlockNodes;
}

bool blockageTest(const std::vector<Building_t*>& buildingSet, Line_t& sd){
  Point_t onfacePoint(-10000.0,-10000.0,-10000.0);
  Point_t* s = sd.getSrc();
  Point_t* d = sd.getDst();
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
    Building_t* curBldg = buildingSet.at(i);
    std::vector<Point_t*> curVgs = curBldg->getVgs();
    std::vector<Point_t*> curVts = curBldg->getVts();
    /* --- Initialize the local variables used to determine the status. --- */
    int faceIntersectCount = 0;
    int sdOnEdgeCount = 0;
    bool topfaceIC = false;
    bool sIsTopV = false;        // true: s is one of the top vertices of the building.
    bool dIsTopV = false;        // true: d is one of the top vertices of the building.
    Point_t* testIntersect;   // A Point_t object to store the intersection point if it exists.
    /* --- Determine whether s or d is one of the top vertices of this building. --- */
    if(s->sameAs(*(curVts.at(0))) || s->sameAs(*(curVts.at(1)))
       || s->sameAs(*(curVts.at(2))) || s->sameAs(*(curVts.at(3)))){
      sIsTopV = true;
    }
    if(d->sameAs(*(curVts.at(0))) || d->sameAs(*(curVts.at(1)))
       || d->sameAs(*(curVts.at(2))) || d->sameAs(*(curVts.at(3)))){
      dIsTopV = true;
    }
    /* --- Test whether the line segement sd intersects with each face of the building. --- */
    /* ------ plane1 is one of the side face of the building. ------ */
    Plane_t* plane1 = new Plane_t(*(curVgs.at(0)), *(curVgs.at(1)), *(curVts.at(1)), *(curVts.at(0)));
    testIntersect = plane1->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      /* sd has one intersection with this face. */
      /* Test whether the intersection point is s or d. */
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
        /* The intersection point is neither s or d of the line segment, thus there is blockage! */
        return true;
      } else{
        /* s or d is the intersection point. Note that, s and d cannot be the same point. */
        faceIntersectCount++; // This face intersect with sd
        /* Test whether this intersection point is on the edge or not. */
        if(std::abs(testIntersect->getZ() - curVts.at(1)->getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(1)->getX()) + std::abs(testIntersect->getY() - curVts.at(1)->getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(0)->getX()) + std::abs(testIntersect->getY() - curVts.at(0)->getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t* plane2 = new Plane_t(*(curVgs.at(1)), *(curVgs.at(2)), *(curVts.at(2)), *(curVts.at(1)));
    testIntersect = plane2->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect->getZ() - curVts.at(1)->getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(1)->getX()) + std::abs(testIntersect->getY() - curVts.at(1)->getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(2)->getX()) + std::abs(testIntersect->getY() - curVts.at(2)->getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t* plane3 = new Plane_t(*(curVgs.at(2)), *(curVgs.at(3)), *(curVts.at(3)), *(curVts.at(2)));
    testIntersect = plane3->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect->getZ() - curVts.at(2)->getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(2)->getX()) + std::abs(testIntersect->getY() - curVts.at(2)->getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(3)->getX()) + std::abs(testIntersect->getY() - curVts.at(3)->getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t* plane4 = new Plane_t(*(curVgs.at(3)), *(curVgs.at(0)), *(curVts.at(0)), *(curVts.at(3)));
    testIntersect = plane4->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
        return true;
      } else{
        faceIntersectCount++;
        if(std::abs(testIntersect->getZ() - curVts.at(0)->getZ()) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(3)->getX()) + std::abs(testIntersect->getY() - curVts.at(3)->getY())) < 1E-5){
          sdOnEdgeCount++;
        } else if((std::abs(testIntersect->getX() - curVts.at(0)->getX()) + std::abs(testIntersect->getY() - curVts.at(0)->getY())) < 1E-5){
          sdOnEdgeCount++;
        }
      }
    }

    Plane_t* plane5 = new Plane_t(*(curVts.at(0)), *(curVts.at(1)), *(curVts.at(2)), *(curVts.at(3)));
    testIntersect = plane5->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
        return true;
      } else{
        faceIntersectCount++;
        Vector_t* AI = new Vector_t(*(curVts.at(0)), *testIntersect);
        Vector_t* AB = new Vector_t(*(curVts.at(0)), *(curVts.at(1)));
        Vector_t* AD = new Vector_t(*(curVts.at(0)), *(curVts.at(3)));
        double Ix = AB->dot(*AI)/AB->mod();  // the x value of p
        double Iy = AD->dot(*AI)/AD->mod();  // the y value of p
        if(std::abs(Ix) <= 1E-5 || std::abs(Ix-AB->mod()) <= 1E-5 || std::abs(Iy) <= 1E-5 || std::abs(Iy-AD->mod()) <= 1E-5){
          sdOnEdgeCount++;
        }
      }
    }
    Plane_t* plane6 = new Plane_t(*(curVgs.at(0)), *(curVgs.at(1)), *(curVgs.at(2)), *(curVgs.at(3)));
    testIntersect = plane6->planeIntersectLine(sd);
    if(testIntersect != nullptr){
      if(!(testIntersect->sameAs(*s) || testIntersect->sameAs(*d))){
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