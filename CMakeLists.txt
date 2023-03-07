cmake_minimum_required(VERSION 2.8.3)
project(pose_graph_tools)

find_package(catkin_simple REQUIRED)

add_compile_options(-std=c++11)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/visualizer.cpp
  src/utils.cpp
  src/types.cpp
)

cs_add_executable(visualizer_node
  src/visualizer_node.cpp
  src/visualizer.cpp
)

cs_install()
cs_export()
