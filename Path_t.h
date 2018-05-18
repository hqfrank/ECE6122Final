//
// Created by qiang on 5/17/18.
//

#ifndef FINALPROJECT_PATH_H
#define FINALPROJECT_PATH_H

#include <vector>

class Path_t {
private:
  int srcId;
  int dstId;
  double maxThroughputMultiHop_Gbps;
  double maxThroughputSingleHop_Gbps;
  int bestPathMultiHopId;
  int bestPathSingleHopId;
  int maxHopNum;

public:
  std::vector<std::vector<int>> pathList;
  std::vector<double> pathThroughput;

  Path_t(int sId, int dId, int maxHop);
  int getSrcId() const;
  int getDstId() const;
  double getMultiHopMaxThroughput() const;
  double getSingleHopMaxThroughput() const;
  void setMultiHopMaxThroughput(double throughput_Gbps);
  void setSingleHopMaxThroughput(double throughput_Gbps);
  int getMultiHopMaxThroughputId() const;
  int getSingleHopMaxThroughputId() const;
  void setMultiHopMaxThroughputId(int pathIndex);
  void setSingleHopMaxThroughputId(int pathIndex);
  int getMaxHopNum() const;


};


#endif //FINALPROJECT_PATH_H
