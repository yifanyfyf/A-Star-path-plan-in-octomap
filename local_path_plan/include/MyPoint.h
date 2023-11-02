#ifndef MYPOINT_H
#define MYPOINT_H

#include <octomap/octomap.h>

class MyPoint: public octomap::point3d{
public:
    MyPoint();
    MyPoint(float &x_, float &y_);
    MyPoint(octomap::point3d* p);
    ~MyPoint(){};

    void SetXY(float &x_, float &y_);
    void SetGHF(float &G_, float &H_, float &F_);
    void SetFather(MyPoint *father_);

    float x, y, z;
    MyPoint* father;
    float G, H, F;
    octomap::point3d* octoPoint;
    bool pointExist = true;
};

#endif