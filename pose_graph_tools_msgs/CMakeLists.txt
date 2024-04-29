cmake_minimum_required(VERSION 3.16)
project(pose_graph_tools_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs
                                        geometry_msgs sensor_msgs)

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

generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
