cmake_minimum_required(VERSION 3.16)
project(pose_graph_tools_msgs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/FeatureVector.msg
  msg/FeatureVectors.msg
  msg/FeatureVectorStamped.msg
  msg/FeatureImage.msg
  srv/EncodeFeature.srv
  DEPENDENCIES
  builtin_interfaces
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY
  msg
  FILES
  BowQueries.msg
  BowQuery.msg
  BowRequests.msg
  BowVector.msg
  LoopClosures.msg
  LoopClosuresAck.msg
  PoseGraph.msg
  PoseGraphEdge.msg
  PoseGraphNode.msg
  TimeRangeQuery.msg
  VLCFrameMsg.msg
  VLCFrames.msg
  VLCRequests.msg)

add_service_files(DIRECTORY srv FILES LcdFrameRegistration.srv
                  PoseGraphQuery.srv VLCFrameQuery.srv)

ament_package()
