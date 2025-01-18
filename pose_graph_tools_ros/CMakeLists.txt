cmake_minimum_required(VERSION 3.16)
project(pose_graph_tools_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pose_graph_tools REQUIRED)
find_package(pose_graph_tools_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)

# find_package(catkin REQUIRED COMPONENTS interactive_markers)

add_library(${PROJECT_NAME} src/conversions.cpp src/visualizer.cpp
                            src/utils.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  geometry_msgs
  pose_graph_tools
  pose_graph_tools_msgs
  rclcpp
  std_msgs
  tf2_eigen
  visualization_msgs)

add_executable(visualizer_node src/visualizer_node.cpp)
target_link_libraries(visualizer_node ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} visualizer_node
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets)
ament_package()
