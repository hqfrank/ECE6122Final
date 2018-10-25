//
// Created by hqfra on 10/23/2018.
//

#include "PhyLink.h"

PhyLink::PhyLink(int linkid, int node1id, int node2id, Point_t n1, Point_t n2) {
    this->linkId = linkid;
    this->node1Id = node1id;
    this->node2Id = node2id;
    this->node1 = n1;
    this->node2 = n2;
    this->dir12 = Vector_t(n1,n2);
    this->dir21 = Vector_t(n2,n1);
}