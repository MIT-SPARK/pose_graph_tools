cmake_minimum_required(VERSION 3.0.2)
project(pose_graph_tools)

find_package(catkin_simple REQUIRED)

add_compile_options(-std=c++11)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/all.cpp
)

cs_install()
cs_export()
