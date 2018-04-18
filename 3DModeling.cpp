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
  std::string str;
  while (std::getline(fileIn, str))
  {
    cout << str << endl;
  }
  File  = new File(); // To store building vertices
  boolean fileExist = fileBuildingVertices.exists();
  Building_t[] buildings;
  try{
    /* Read all lines in the file. */
    List<String> filelines = Files.readAllLines(Paths.get(dataBuildings), StandardCharsets.UTF_8);
    int numLines = filelines.size();
    System.out.println(numLines+"\tbuildings");
    buildings = new Building_t[numLines];
    int index = 0;
    while(index < numLines){
      String[] data = filelines.get(index).split("\t");
      double[] center = new double[]{Double.parseDouble(data[0]),Double.parseDouble(data[1])};
      double length_m = Double.parseDouble(data[2]);
      double width_m = Double.parseDouble(data[3]);
      double topHeight_m = Double.parseDouble(data[4]);
      double baseLevel_m = Double.parseDouble(data[5]);
      double[] lwhbg = new double[]{length_m, width_m, topHeight_m, baseLevel_m + parameters.minHeightForRelay_m, parameters.groundLevel_m};
      double orientation_rad = degreeToRad(Double.parseDouble(data[6]));
      /* Generate each building object. */
      buildings[index] = new Building_t(center, lwhbg, orientation_rad, parameters.maxHeightForRelay_m, parameters.densityRelayOnBuilding, parameters.randomSeed);
      /* Write building vertices information to file. */
      if (!fileExist){
        PrintWriter printWriter = new PrintWriter(new FileOutputStream(dataBuildingVertices, true));
        printWriter.println(buildings[index].toString());
        printWriter.close();
      }
      index++;
    }
    return buildings;
  } catch (IOException e){
    return null;
  }
}
