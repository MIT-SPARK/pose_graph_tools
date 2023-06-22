cmake_minimum_required(VERSION 3.0.2)
project(pose_graph_tools_msgs)

find_package(catkin_simple REQUIRED)

add_compile_options(-std=c++11)

catkin_simple()

cs_install()
cs_export()
