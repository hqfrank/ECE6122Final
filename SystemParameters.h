//
// Created by qiang on 4/18/18.
//

#ifndef FINALPROJECT_SYSTEMPARAMETERS_H
#define FINALPROJECT_SYSTEMPARAMETERS_H

#include <string>
#include <cmath>

class SystemParameters {
public:
  const int randomSeed = 10;
  const std::string relayType = "Top";
  const double minHeightForRelay_m = 5.0;
  const double maxHeightForRelay_m = 200.0;
  const double minHeightForBS_m = 10.0; // Only deploy BSs on the rooftop of buildings with height between [min,max].
  const double maxHeightForBS_m = 50.0;
  const double groundLevel_m = 290.0;
  const double densityRelayOnBuilding = 0.00005;
  const double areaXRange_m[2] {-100, 1700};
  const double areaYRange_m[2] {-1700, 100};
  const double gridSize_m = 100.0;
  const int maxNumRelaysInGrid = 1;
  const double maxLengthLOSLinkBSs_m = 100.0;
  const double phyLinkDistMax_m = 300;
  const double bsDistanceRange_m[2] {20,200};
  const int numBSPairs = 100;
  const double antennaGain_dBi = 21.87;
  const double antennaBeamWidth_phi = 5.0/180.0*M_PI;
  const double lambda_m = 0.005;   // 5*10^-3 m
  const double alpha = 16E-3;
  const double noise_dBm = 10.0 * log10(1.38064852E-11 * 290 * 2.16);
  const double pt_w = 1.0;    // 1 * 10^0 watt
  const double exponent = 2.0;
  const double linkMargin_dB = 10;
  const int hopLimit = 10; // the maximum hop number allowed
  double lowerBound_Gbps = 0;
};


#endif //FINALPROJECT_SYSTEMPARAMETERS_H