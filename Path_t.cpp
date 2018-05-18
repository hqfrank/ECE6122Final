//
// Created by qiang on 5/17/18.
//

#include "Path_t.h"

Path_t::Path_t(int sId, int dId, int maxHop) {
  srcId = sId;
  dstId = dId;
  maxThroughputSingleHop_Gbps = 0;
  maxThroughputMultiHop_Gbps = 0;
  bestPathSingleHopId = -1;
  bestPathMultiHopId = -1;
  maxHopNum = maxHop;


  pathList = std::vector<std::vector<int>>();
  pathThroughput = std::vector<double>();

}

int Path_t::getSrcId() const {
  return srcId;
}

int Path_t::getDstId() const {
  return dstId;
}

double Path_t::getMultiHopMaxThroughput() const {
  return maxThroughputMultiHop_Gbps;
}

double Path_t::getSingleHopMaxThroughput() const {
  return maxThroughputSingleHop_Gbps;
}

void Path_t::setMultiHopMaxThroughput(double throughput_Gbps) {
  maxThroughputMultiHop_Gbps = throughput_Gbps;
}

void Path_t::setSingleHopMaxThroughput(double throughput_Gbps) {
  maxThroughputSingleHop_Gbps = throughput_Gbps;
}

int Path_t::getMultiHopMaxThroughputId() const {
  return bestPathMultiHopId;
}

int Path_t::getSingleHopMaxThroughputId() const {
  return bestPathSingleHopId;
}

void Path_t::setMultiHopMaxThroughputId(int pathIndex) {
  bestPathMultiHopId = pathIndex;
}

void Path_t::setSingleHopMaxThroughputId(int pathIndex) {
  bestPathSingleHopId = pathIndex;
}

int Path_t::getMaxHopNum() const {
  return maxHopNum;
}