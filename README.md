# A-Star-path-plan-in-octomap

A simple example to show how to use A Star algorithm to do path plan in octomap.
Run scritp/build_map to build a map.
astar_py.py is a demo to show the principle of A Star algorithm.
In main.cpp function ReadTxt() and convertVectorToOctree() convert the map built by python to octomap. When optimizing, the octomap is published to ros topic and it can be visualizeb by RVIZ.
