cmake_minimum_required(VERSION 3.0.2)
project(pose_graph_tools_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_install()
cs_export()
