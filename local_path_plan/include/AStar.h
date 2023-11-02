#ifndef ASTAR_H
#define ASTAR_H

#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <vector>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "ros/ros.h"

#include "MyPoint.h"


class AStar{
public:
    AStar(octomap::OcTree *map_, float &startX, float &startY, float &endX_, float &endY_);
    ~AStar(){};
    bool GetNodeCoord(float &x_, float &y_, float &nodeCoordX, float &nodeCoordY);
    void CalGHF(MyPoint* checkingPoint, float &G, float &H, float &F);
    std::vector<MyPoint*> GetNearPoints(MyPoint *checkingPoint);
    MyPoint* IsInOpenList(MyPoint *pi);
    void Add2OpenList(MyPoint *pi);     
    MyPoint* ChooseMinFPoint();
    void EraseOnePoint(MyPoint* ptr);
    
    bool IsEqual(MyPoint* p1, MyPoint* p2);
    float CalDistance(MyPoint* a, MyPoint* b);
    bool Run();

    octomap::OcTree *map;
    float resolution;
    std::vector<MyPoint*> openList;
    std::vector<MyPoint*> closeList;
    float endZ;
    float nodeCoordX_endPoint, nodeCoordY_endPoint;
    MyPoint* endPoint;
    MyPoint* startPoint;
    std::vector<MyPoint*> path;
    MyPoint* checkingPoint = nullptr;
    bool succeed = false;
};

#endif
