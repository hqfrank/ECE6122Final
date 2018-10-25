//
// Created by hqfra on 10/23/2018.
//

#ifndef FINALPROJECT_PHYLINK_H
#define FINALPROJECT_PHYLINK_H

#include "Point_t.h"

class PhyLink {
public:
    int linkId;  // the index among all physical links in the network
    int node1Id; // the index of node1
    int node2Id; // the index of node2
    Point_t node1;  // node1
    Point_t node2;  // node2
    Vector_t dir12; // the direction from node1 to node 2;
    Vector_t dir21; // the direction from node2 to node 1;

    PhyLink(int linkid, int node1id, int node2id, Point_t n1, Point_t n2);

};


#endif //FINALPROJECT_PHYLINK_H
