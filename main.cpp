#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <chrono>
#include <ctime>
#include "Point_t.h"
#include "Vector_t.h"
#include "Line_t.h"
#include "3DModeling.h"
#include "Plane_t.h"
#include "Building_t.h"
#include "SystemParameters.h"

using namespace std;

int main() {
  std::cout << "Hello, World!" << std::endl;
  Point_t p1(3,10,15);
  Point_t p2(10,15,2);
  cout << p1.toString() << endl;
  cout << p2.toString() << endl;
  Vector_t v1(p1, p2);
  cout << v1.toString() << endl;
  Line_t l1(p1, p2);
  cout << l1.toString() << endl;
  Point_t p3(5,13,19);
  Point_t p4(12,18,6);
  cout << p3.toString() << endl;
  cout << p4.toString() << endl;
  Plane_t pl1(p1, p2, p3, p4);
  cout << pl1.toString() << endl;
  Point_t center(0,0,0);
  double c[] {center.getX(), center.getY()};
  double lwhbg[] {40, 30, 60, 4, -1};
  double o = 45 / 180.0 * M_PI;
  double t = 100;
  double d = 0.01;
  double r = 1;
  Building_t b1(c, lwhbg, o, t, d, r);
  cout << b1.toString() << endl;
  cout << b1.getRelays().size() << endl;
  cout << b1.getRelays().at(0)->toString() << endl;
  cout << b1.getRelays().at(b1.getRelays().size() - 1)->toString() << endl;
//  std::chrono::system_clock::time_point t0 = std::chrono::system_clock::now();
//  std::time_t strTime = chrono::system_clock::to_time_t(t0);
//  cout << ctime(&strTime) << endl;

  /*
   *  MAIN FUNCTION
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
  std::string strDataBuildingVertices = "../Data/Output/Building_Vertices/Data_BuildingVertices_" + std::to_string(sysParams.randomSeed) +".txt";

  /*
   * ====================================
   * Construct all buildings in the area.
   * ====================================
   */
  std::ifstream file(strDataBuildings);
  std::string str;
  while (std::getline(file, str))
  {
    cout << str << endl;
  }


  return 0;
}