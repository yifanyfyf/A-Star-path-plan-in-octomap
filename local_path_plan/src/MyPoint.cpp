#include "MyPoint.h"


MyPoint::MyPoint(){
    father = nullptr;
    z = 0;
}

MyPoint::MyPoint(float &x_, float &y_){
    x = x_;
    y = y_;
    z = 0;
    father = nullptr;
};

MyPoint::MyPoint(octomap::point3d* p){
    x = p->x();
    y = p->y();
    father = nullptr;
    octoPoint = p;
};



void MyPoint::SetXY(float &x_, float &y_){
    x = x_;
    y = y_;
};

void MyPoint::SetGHF(float &G_, float &H_, float &F_){
    G = G_;
    H = H_;
    F = F_;
};

void MyPoint::SetFather(MyPoint *father_){
    father = father_;
}