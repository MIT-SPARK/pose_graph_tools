cmake_minimum_required(VERSION 3.16)
project(pose_graph_tools_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

find_package(pose_graph_tools REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             tf2_eigen
             pose_graph_tools_msgs
             std_msgs
             geometry_msgs
             visualization_msgs
             interactive_markers)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf2_eigen
  pose_graph_tools_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  interactive_markers
  DEPENDS
  pose_graph_tools
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})

add_library(${PROJECT_NAME} src/conversions.cpp src/visualizer.cpp
                            src/utils.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC pose_graph_tools::pose_graph_tools
                                             ${catkin_LIBRARIES})

add_executable(visualizer_node src/visualizer_node.cpp)
target_link_libraries(visualizer_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
