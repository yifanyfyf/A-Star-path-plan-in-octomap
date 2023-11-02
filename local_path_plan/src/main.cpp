#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeBase.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "ros/ros.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include "AStar.h"


void PublishThread(octomap::OcTree* octree, AStar* aStar, float startX, float startY, float endX, float endY);

void AStarStart(octomap::OcTree* octree);

std::vector<std::vector<int>> ReadTxt();

void convertVectorToOctree(const std::vector<std::vector<int>>& map_array);

int main(int argc, char **argv){
    // 初始化ROS节点
    ros::init(argc, argv, "orbslam_pointcloud_subscriber");

    // 创建一个OctoMap的OcTree对象
    octomap::OcTree* octree = new octomap::OcTree(1.0);
    if (!octree->readBinary("./data/my_octree.bt")){
        std::cerr << "Failed to load the octree file." << std::endl;
        return 1;
    }

    AStarStart(octree);

    // std::vector<std::vector<int>> map_array = ReadTxt();
    // convertVectorToOctree(map_array);

    return 0;
}

void AStarStart(octomap::OcTree* octree){
    float startX = 3.0;
    float startY = 0.0;
    float endX = 22.0;
    float endY = 4.0;
    AStar* aStar = new AStar(octree, startX, startY, endX, endY);

    // PublishThread(octree, aStar, startX, startY, endX, endY); // reset the map color

    while (!aStar->openList.empty() && !aStar->succeed){
        aStar->Run();
        // PublishThread(octree, aStar, startX, startY, endX, endY);   // for obeserving one step
    }
    PublishThread(octree, aStar, startX, startY, endX, endY);
}

void PublishThread(octomap::OcTree* octree, AStar* aStar, float startX, float startY, float endX, float endY){
    // 创建ROS节点句柄
    ros::NodeHandle nh;
    std::string topic, frame_id;
    int hz;
    nh.param<std::string>("frame_id", frame_id, "map");
    nh.param<std::string>("topic", topic, "/octree");
    nh.param<int>("hz", hz, 10);
    // Create a publisher for the Octomap topic
    ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>(topic, 1);
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.frame_id  =frame_id;

    // 创建一个 ColorOcTree 对象
    octomap::ColorOcTree color_octree(octree->getResolution());
    // 设置所有节点为绿色
    for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            octomap::ColorOcTreeNode::Color green_color(0, 255, 0);  // 绿色
            octomap::ColorOcTreeNode* color_node = color_octree.updateNode(it.getKey(), true);
            color_node->setColor(green_color);
        }
    }

    // 设置一些节点可用的颜色
    octomap::ColorOcTreeNode::Color yellow_color(255, 255, 0);  // 黄色 起点
    octomap::ColorOcTreeNode::Color blue_color(0, 0, 255);  // 蓝色 终点
    octomap::ColorOcTreeNode::Color purple_color(128, 0, 128);  // 紫色 路径
    octomap::ColorOcTreeNode::Color sky_color(0, 255, 255);  // 青色 开列表
    octomap::ColorOcTreeNode::Color gray_color(125, 125, 125);  // 灰色 闭列表
    octomap::ColorOcTreeNode::Color red_color(255, 0, 0);  // 红色 checkingpoint

    octomap::point3d star_point(startX, startY, 0.5);
    octomap::point3d end_point(endX, endY, 0.0);

    // 设置起点 终点颜色
    octomap::ColorOcTreeNode* yellow_node2 = color_octree.updateNode(star_point, false);
    yellow_node2->setColor(yellow_color);
    octomap::ColorOcTreeNode* blue_node2 = color_octree.updateNode(end_point, false);
    blue_node2->setColor(blue_color);

    if (!aStar->openList.empty() && !aStar->succeed){
        // 设置开列表颜色
        for (MyPoint* pi : aStar->openList) {
            octomap::point3d tmp_point(pi->x, pi->y, 0.5);
            octomap::ColorOcTreeNode* path_node = color_octree.updateNode(tmp_point, false);
            path_node->setColor(sky_color);
        }
        // 设置闭列表颜色
        for (MyPoint* pi : aStar->closeList) {
            octomap::point3d tmp_point(pi->x, pi->y, 0.5);
            octomap::ColorOcTreeNode* path_node = color_octree.updateNode(tmp_point, false);
            path_node->setColor(gray_color);
        }
    }
    else{
        // 设置路径颜色
        for (MyPoint* pi : aStar->path) {
            octomap::point3d tmp_point(pi->x, pi->y, 0.5);
            octomap::ColorOcTreeNode* path_node = color_octree.updateNode(tmp_point, false);
            path_node->setColor(purple_color);
        }
    }   

    // Publish the Octomap
    ros::Rate loop_rate(hz);  // Adjust the p5ublishing rate as needed
    for (int i=0; i < 5e9; i++) {     // for observing each step
        octomap_msgs::fullMapToMsg(color_octree, octomap_msg);
        octomap_msg.header.stamp=ros::Time::now();
        octomap_msg.header.frame_id  =frame_id;
        std::cout<<"time stamp = "<<octomap_msg.header.stamp<<std::endl;
        std::cout<<"frame_id = "<<octomap_msg.header.frame_id<<std::endl;
        octomap_pub.publish(octomap_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}


std::vector<std::vector<int>> ReadTxt() {
    const std::string file_path = "./script/my_map222.txt";

    // 打开文件
    std::ifstream file(file_path);

    if (!file.is_open()) {
        std::cerr << "无法打开文件 " << file_path << std::endl;
    }

    // 读取文件中的二维数组
    std::vector<std::vector<int>> values;
    std::string line;

    while (std::getline(file, line)) {
        std::vector<int> row;
        std::stringstream ss(line);
        int value;

        while (ss >> value) {
            row.push_back(value);
            if (ss.peek() == ',')
                ss.ignore();
        }

        values.push_back(row);
    }

    // 关闭文件
    file.close();

    // 输出读取到的二维数组
    for (const auto& row : values) {
        for (int v : row) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
    }

    return values;
}

void convertVectorToOctree(const std::vector<std::vector<int>>& map_array) {
    // 设置分辨率和树深度
    double resolution = 1; // 设置分辨率为0.1米

    // 创建 Octree
    octomap::OcTree octree(resolution);
    // 遍历二维数组，将值映射到 Octree 中
    for (size_t y = 0; y < map_array.size(); ++y) {
        for (size_t x = 0; x < map_array[y].size(); ++x) {
            // 根据二维数组中的值来设置节点状态
            double x_coord = x * resolution;
            double y_coord = y * resolution;
            double z_coord = 0;
            
            if (map_array[y][x] == 1) {
                // 设置节点为占用状态
                std::cout << y << x << std::endl;
                octree.updateNode(x_coord, y_coord, z_coord, true);
            }
        }
    }
    if(octree.writeBinary("./data/my_octree_simple.bt")){
        std::cout << "succeed" << std::endl;
    }
    else{
        std::cout << "failed" << std::endl;
    }
    
}


