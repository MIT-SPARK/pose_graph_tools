cmake_minimum_required(VERSION 3.0.2)
project(pose_graph_tools_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
  src/conversions.cpp
  src/visualizer.cpp
  src/utils.cpp
)

cs_add_executable(visualizer_node
  src/visualizer_node.cpp
  src/visualizer.cpp
)

cs_install()
cs_export()
