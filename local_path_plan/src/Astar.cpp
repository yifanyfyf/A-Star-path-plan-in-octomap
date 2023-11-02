#include "AStar.h"


bool AStar::GetNodeCoord(float &x_, float &y_, float &nodeCoordX, float &nodeCoordY){
    int result_x = static_cast<int>(x_ / resolution); 
    float floatResult_x = static_cast<float>(result_x);
    int result_y = static_cast<int>(y_ / resolution); 
    float floatResult_y = static_cast<float>(result_y);

    float z_ = 0;
    octomap::point3d min_bound(result_x + 0.25*resolution, result_y + 0.25*resolution, z_-resolution);
    octomap::point3d max_bound(result_x + 0.75*resolution, result_y + 0.75*resolution, z_ + resolution);
    octomap::OcTree::leaf_bbx_iterator it = map->begin_leafs_bbx(min_bound,max_bound);
    octomap::OcTree::leaf_bbx_iterator end = map->end_leafs_bbx();
    while (it != end){
        octomap::point3d node_coordinate = it.getCoordinate();
        nodeCoordX = node_coordinate.x();
        nodeCoordY = node_coordinate.y();
        return true;
    }
    return false;

}

AStar::AStar(octomap::OcTree *map_, float &startX, float &startY, float &endX_, float &endY_){
    map = map_;
    resolution = map_->getResolution();
    // 计算起点和终点的voxel坐标
    float nodeCoordX, nodeCoordY;
    bool getNode = AStar::GetNodeCoord(startX, startY, nodeCoordX, nodeCoordY);
    float endX = endX_;
    float endY = endY_;
    endZ = 0;
    if(AStar::GetNodeCoord(endX, endY, nodeCoordX_endPoint, nodeCoordY_endPoint)){
        endPoint = new MyPoint(nodeCoordX_endPoint, nodeCoordY_endPoint);
    }

    if(getNode) {
        startPoint = new MyPoint(nodeCoordX, nodeCoordY);
    
        float G, H, F;
        AStar::CalGHF(startPoint, G, H, F);
        startPoint->SetGHF(G, H, F);
        AStar::Add2OpenList(startPoint);
    }
    
};

void AStar::CalGHF(MyPoint* calPoint, float &G, float &H, float &F){
    if (calPoint->father != nullptr){
        float x_father = calPoint->father->x;
        float y_father = calPoint->father->y;
        float deltaX = calPoint->x - calPoint->father->x;
        float deltaY = calPoint->y - calPoint->father->y;
        float distance = std::sqrt(deltaX * deltaX + deltaY * deltaY);
        G = calPoint->father->G + distance;
    }
    else{
        G = 0;
    }

    H = std::abs(calPoint->x - nodeCoordX_endPoint) + std::abs(calPoint->y - nodeCoordY_endPoint);
    F = G + H;
};

float AStar::CalDistance(MyPoint* a, MyPoint* b){
    float deltaX = a->x - b->x;
    float deltaY = a->y - b->y;
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
};

// 自定义比较函数，用于比较 MyPoint 对象的 x 属性
bool AStar::IsEqual(MyPoint* p1, MyPoint* p2) {
    return (p1->x == p2->x && p1->y == p2->y);
};

std::vector<MyPoint*> AStar::GetNearPoints(MyPoint* checkingPoint){
    float xMinus1 = checkingPoint->x - resolution;
    float x = checkingPoint->x;
    float xPlus1 = checkingPoint->x + resolution;
    float yMinus1 = checkingPoint->y - resolution;
    float y = checkingPoint->y;
    float yPlus1 = checkingPoint->y + resolution;

    octomap::point3d np0(xMinus1, yMinus1, endZ);
    octomap::point3d np1(xMinus1, y, endZ);
    octomap::point3d np2(xMinus1, yPlus1, endZ);
    octomap::point3d np3(x, yMinus1, endZ);
    octomap::point3d np4(x, yPlus1, endZ);
    octomap::point3d np5(xPlus1, yMinus1, endZ);
    octomap::point3d np6(xPlus1, y, endZ);
    octomap::point3d np7(xPlus1, yPlus1, endZ);
    std::vector<octomap::point3d> nearPointsList_ = {np0, np1, np2, np3, np4, np5, np6, np7};
    std::vector<MyPoint*> nearPointsList;

    //判断是否在可行使区域内
    for (std::vector<octomap::point3d>::iterator it = nearPointsList_.begin(); it != nearPointsList_.end(); ++it) {
        octomap::OcTreeNode* node = map->search(*it);
        if(node){
            // 得到这个邻近点对应的octree节点的中心坐标，用这个坐标生成一个MyPoint
            float nodeCoordX, nodeCoordY;
            if(AStar::GetNodeCoord((*it).x(), (*it).y(), nodeCoordX, nodeCoordY)){
                MyPoint* tmpMp = new MyPoint(nodeCoordX, nodeCoordY);
                // 判断是否在闭列表中
                bool found = false;
                for (MyPoint* point : closeList) {
                    if (AStar::IsEqual(point, tmpMp)) {
                        found = true;
                        break;
                    }
                }
                if(!found) nearPointsList.push_back(tmpMp);
            }
        }
        else{
            // std::cout << "not exist" << std::endl;
        }
    }

    return nearPointsList;
};

MyPoint* AStar::IsInOpenList(MyPoint *pi){
    MyPoint* ptr2OpenList = nullptr;
    for (MyPoint* point : openList) {
        if (AStar::IsEqual(point, pi)) {
            ptr2OpenList = point;
            break;
        }
    }
    return ptr2OpenList;
};

void AStar::Add2OpenList(MyPoint *pi){
    openList.push_back(pi);
};

MyPoint* AStar::ChooseMinFPoint(){
    MyPoint* minFPoint = openList[0]; // 假设第一个元素的F值最小

    for (size_t i = 1; i < openList.size(); ++i) {
        if (openList[i]->F < minFPoint->F) {
            minFPoint = openList[i]; // 找到F值更小的元素
        }
    }

    return minFPoint;
}

void AStar::EraseOnePoint(MyPoint* ptr){
    auto it = std::find(openList.begin(), openList.end(), ptr);

    if (it != openList.end()) {
        // Element found, erase it
        openList.erase(it);
    } else {
        std::cout << "Element not found in the vector." << std::endl;
    }

}

bool AStar::Run(){
    if (!openList.empty() && !succeed){
        checkingPoint = AStar::ChooseMinFPoint();
        std::vector<MyPoint*> nearPointsList = AStar::GetNearPoints(checkingPoint);
        AStar::EraseOnePoint(checkingPoint);
        closeList.push_back(checkingPoint);
        for (MyPoint* pi : nearPointsList){
            // 到达目标点
            if (AStar::IsEqual(pi, endPoint)){

                path.push_back(endPoint);
                path.push_back(checkingPoint);
                MyPoint* father = checkingPoint->father;
                while(father != nullptr){
                    path.push_back(father);
                    father = father->father;
                }
                succeed = true;
                return true;
            }
            // 没到达目标点
            // 判断nearPointsList中的这个pi是不是跟开列表中已有的点类实例具有相同的坐标
            MyPoint* ptr2OpenList = AStar::IsInOpenList(pi);
            if (ptr2OpenList == nullptr){    // 不在
                pi->father = checkingPoint;
                float G, H, F;
                AStar::CalGHF(pi, G, H, F);
                pi->SetGHF(G, H, F);
                AStar::Add2OpenList(pi);
            }
            // 在开列表中
            else{
                float G_new = checkingPoint->G + AStar::CalDistance(checkingPoint, ptr2OpenList);
                if (G_new < ptr2OpenList->G){
                    ptr2OpenList->father = checkingPoint;
                    float G, H, F;
                    AStar::CalGHF(ptr2OpenList, G, H, F);
                    ptr2OpenList->SetGHF(G, H, F);
                }
            }
        }
    }
        
    return false;
};