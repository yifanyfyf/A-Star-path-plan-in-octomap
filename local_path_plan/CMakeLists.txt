cmake_minimum_required(VERSION 2.8)
 
project(local_path_plan)
 
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  octomap_msgs
)

find_package(octomap REQUIRED)

include_directories(
./include
${catkin_INCLUDE_DIRS}
${OCTOMAP_INCLUDE_DIRS}
)
 
set(SRC_LIST
./src/main.cpp
./src/MyPoint.cpp
./src/Astar.cpp
)
 
add_executable(main ${SRC_LIST})

target_link_libraries( main 
${catkin_LIBRARIES}
${OCTOMAP_LIBRARIES}
)