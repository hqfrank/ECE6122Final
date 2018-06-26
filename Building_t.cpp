//
// Created by qiang on 4/16/18.
//

#include "Building_t.h"

Building_t::Building_t(double center[2], double lwhbg[5], double orientation, double threshold, double density, int randomSeed){
  this->Center = Point_t(center[0], center[1], 0);
  this->Length = lwhbg[0];
  this->Width  = lwhbg[1];
  this->Height = lwhbg[2];
  this->HeightBase = lwhbg[3];
  this->GroundLevel = lwhbg[4];
  this->Orientation = orientation;
  VertexGenerator();
  this->Relays = RelayGenerator(threshold, density, randomSeed);
  this->NumRelays = Relays.size();
  this->hasBS = false;
}

void Building_t::VertexGenerator() {
  double c1[] {
    this->Center.getX() + this->Width/2*std::cos(this->Orientation),
    this->Center.getY() - this->Width/2*std::sin(this->Orientation)
  };
  double c2[] {
    this->Center.getX() + this->Length/2*std::sin(this->Orientation),
    this->Center.getY() + this->Length/2*std::cos(this->Orientation)
  };
  double c3[] {
    this->Center.getX() - this->Width/2*std::cos(this->Orientation),
    this->Center.getY() + this->Width/2*std::sin(this->Orientation)
  };
  double c4[] {
    this->Center.getX() - this->Length/2*std::sin(this->Orientation),
    this->Center.getY() - this->Length/2*std::cos(this->Orientation)
  };
  // Calculate the 4 vertices at the base level
  Point_t VAb(c1[0]+c4[0]-this->Center.getX(), c1[1]+c4[1]-this->Center.getY(), this->HeightBase);
  this->Vbs.push_back(VAb); // add VAb
  Point_t VBb(c1[0]+c2[0]-this->Center.getX(), c1[1]+c2[1]-this->Center.getY(), this->HeightBase);
  this->Vbs.push_back(VBb); // add VBb
  Point_t VCb(c2[0]+c3[0]-this->Center.getX(), c2[1]+c3[1]-this->Center.getY(), this->HeightBase);
  this->Vbs.push_back(VCb); // add VCb
  Point_t VDb(c3[0]+c4[0]-this->Center.getX(), c3[1]+c4[1]-this->Center.getY(), this->HeightBase);
  this->Vbs.push_back(VDb); // add VDb
  // Calculate the 4 vertices at the ground level
  Point_t VAg(c1[0]+c4[0]-this->Center.getX(), c1[1]+c4[1]-this->Center.getY(), this->GroundLevel);
  this->Vgs.push_back(VAg); // add VAg
  Point_t VBg(c1[0]+c2[0]-this->Center.getX(), c1[1]+c2[1]-this->Center.getY(), this->GroundLevel);
  this->Vgs.push_back(VBg); // add VBg
  Point_t VCg(c2[0]+c3[0]-this->Center.getX(), c2[1]+c3[1]-this->Center.getY(), this->GroundLevel);
  this->Vgs.push_back(VCg); // add VCg
  Point_t VDg(c3[0]+c4[0]-this->Center.getX(), c3[1]+c4[1]-this->Center.getY(), this->GroundLevel);
  this->Vgs.push_back(VDg); // add VDg
  // Calculate the 4 vertices at the top level
  Point_t VAt(c1[0]+c4[0]-this->Center.getX(), c1[1]+c4[1]-this->Center.getY(), this->Height);
  this->Vts.push_back(VAt); // add VAt
  Point_t VBt(c1[0]+c2[0]-this->Center.getX(), c1[1]+c2[1]-this->Center.getY(), this->Height);
  this->Vts.push_back(VBt); // add VBt
  Point_t VCt(c2[0]+c3[0]-this->Center.getX(), c2[1]+c3[1]-this->Center.getY(), this->Height);
  this->Vts.push_back(VCt); // add VCt
  Point_t VDt(c3[0]+c4[0]-this->Center.getX(), c3[1]+c4[1]-this->Center.getY(), this->Height);
  this->Vts.push_back(VDt); // add VDt

  // Try to use openGL to draw the figure.
//  StdDraw.setPenRadius(0.001);
//  StdDraw.setPenColor(StdDraw.BLACK);
//  StdDraw.line(VAb.x,VAb.y,VBb.x,VBb.y);
//  StdDraw.line(VBb.x,VBb.y,VCb.x,VCb.y);
//  StdDraw.line(VCb.x,VCb.y,VDb.x,VDb.y);
//  StdDraw.line(VDb.x,VDb.y,VAb.x,VAb.y);
}


std::vector<Point_t> Building_t::getVbs() const {
  return this->Vbs;
}

std::vector<Point_t> Building_t::getVgs() const {
  return this->Vgs;
}

std::vector<Point_t> Building_t::getVts() const {
  return this->Vts;
}

std::vector<Point_t> Building_t::getRelays() const {
  return this->Relays;
}

double Building_t::getHeight() const {
  return this->Height;
}

double Building_t::getHeightBase() const {
  return this->HeightBase;
}

void Building_t::setHasBS(bool has) {
  this->hasBS = has;
}

std::vector<Point_t> Building_t::GenerateRelayOnFace(const Point_t& va, const Point_t& vb, const Point_t& vc, const Point_t& vd, double density, int randomSeed){
  // va and vb are two bottom vertices, vd and vc are two top vertices, and (va->vb).(vb->vc) = 0
  /*
   *   (vd) *--------* (vc)
   *        |        |
   *        |        | <-- side2
   *        |        |
   *   (va) *--------* (vb)
   *           side1
   */
  // Calculate the length of the two sides of the rectangular building face.
  double side1 = sqrt( pow(va.getX()-vb.getX(), 2.0) + pow(va.getY()-vb.getY(), 2.0) + pow(va.getZ()-vb.getZ(), 2.0) );
  double side2 = sqrt( pow(vb.getX()-vc.getX(), 2.0) + pow(vb.getY()-vc.getY(), 2.0) + pow(vb.getZ()-vc.getZ(), 2.0) );
  // Every face has at least one relay, the number of additional relays are generated based on density.
  int numRelayOnFace = (int) floor(density * side1 * side2) + 1;
  // Initialize relayPosOnFace to store the pointers to each relay node.
  std::vector<Point_t> relayPosOnFace;
  // Random generator
  // Seed with a real random value, if available
  std::random_device r;
  std::default_random_engine e(randomSeed);
  std::uniform_int_distribution<int> uniform_dist(0, 999);
  for (int i = 0; i < numRelayOnFace; i++) {
    // the project of this relay on side va->vb is located side1*ratioab away from node va
    // the project of this relay on side va->vd is located side2*ratioad away from node va
    uniform_dist(e);
    double ratioab = uniform_dist(e)/1000.0;
    double ratioad = uniform_dist(e)/1000.0;
    double pab[] {
      va.getX() + ratioab * (vb.getX() - va.getX()),
      va.getY() + ratioab * (vb.getY() - va.getY()),
      va.getZ() + ratioab * (vb.getZ() - va.getZ())
    };
    double pad[] {
      va.getX() + ratioad * (vd.getX() - va.getX()),
      va.getY() + ratioad * (vd.getY() - va.getY()),
      va.getZ() + ratioad * (vd.getZ() - va.getZ())
    };
    Point_t newRelay( pab[0] + pad[0] - va.getX(), pab[1] + pad[1] - va.getY(), pab[2] + pad[2] - va.getZ());
    relayPosOnFace.push_back(newRelay);
  }
  return relayPosOnFace;
}

std::vector<Point_t> Building_t::RelayGenerator(double threshold, double density, int randomSeed) {
  // {Ab, Bb, Bt, At}, {Bb, Cb, Ct, Bt}, {Cb, Db, Dt, Ct}, {Db, Ab, At, Dt}, and {At, Bt, Ct, Dt}
  std::vector<Point_t> relayF1 = GenerateRelayOnFace((this->Vbs.at(0)), (this->Vbs.at(1)), (this->Vts.at(1)), (this->Vts.at(0)), density, randomSeed * 5 + 1);
  std::vector<Point_t> relayF2 = GenerateRelayOnFace((this->Vbs.at(1)), (this->Vbs.at(2)), (this->Vts.at(2)), (this->Vts.at(1)), density, randomSeed * 5 + 2);
  std::vector<Point_t> relayF3 = GenerateRelayOnFace((this->Vbs.at(2)), (this->Vbs.at(3)), (this->Vts.at(3)), (this->Vts.at(2)), density, randomSeed * 5 + 3);
  std::vector<Point_t> relayF4 = GenerateRelayOnFace((this->Vbs.at(3)), (this->Vbs.at(0)), (this->Vts.at(0)), (this->Vts.at(3)), density, randomSeed * 5 + 4);
  std::vector<Point_t> relayF5;
  // There must be relays on the 4 side faces
  int numRelayOnBuilding = relayF1.size() + relayF2.size() + relayF3.size() + relayF4.size();
  /*
   * Only add relays on the building top when the effective height of the building
   * (i.e., the level difference between the top and the base) is shorter than
   * the threshold.
   */
  if ((this->Height - this->HeightBase) <= threshold){
    relayF5 = GenerateRelayOnFace((this->Vts.at(0)), (this->Vts.at(1)), (this->Vts.at(2)), (this->Vts.at(3)), density, randomSeed * 5 + 5);
    numRelayOnBuilding += relayF5.size();
  }

  std::vector<Point_t> relayPosOnBuilding;
//  /* Relays are drawn in blue color. */
//  StdDraw.setPenRadius(0.005);
//  StdDraw.setPenColor(StdDraw.BLUE);
  /* Add each relay on different faces to the building. */
  relayF1.insert(relayF1.end(), relayF2.begin(), relayF2.end());
  relayF1.insert(relayF1.end(), relayF3.begin(), relayF3.end());
  relayF1.insert(relayF1.end(), relayF4.begin(), relayF4.end());
  if ((this->Height - this->HeightBase) <= threshold){
    relayF1.insert(relayF1.end(), relayF5.begin(), relayF5.end());
  }

  assert(relayF1.size() == numRelayOnBuilding);

  return relayF1;
}

std::string Building_t::toString() const {
  return "Building:\nVbs: " + this->Vbs.at(0).toString() + ", " + this->Vbs.at(1).toString() + ", " +
    this->Vbs.at(2).toString() + ", " + this->Vbs.at(3).toString() + "\n" +
    "Vts: " + this->Vts.at(0).toString() + ", " + this->Vts.at(1).toString() + ", " + this->Vts.at(2).toString() +
    ", " + this->Vts.at(3).toString();
}

std::string Building_t::toStringData() const {
  return this->Vbs.at(0).toStringData() + "\t" + this->Vbs.at(1).toStringData() + "\t" +
    this->Vbs.at(2).toStringData() + "\t" + this->Vbs.at(3).toStringData() + "\t" +
    this->Vts.at(0).toStringData() + "\t" + this->Vts.at(1).toStringData() + "\t" +
    this->Vts.at(2).toStringData() + "\t" + this->Vts.at(3).toStringData();
}