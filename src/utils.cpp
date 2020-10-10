/*
 * Copyright Notes
 *
 * Authors: Yulun Tian (yulun@mit.edu)
 */

#include <pose_graph_tools/utils.h>
#include <ros/console.h>

#include <fstream>

namespace pose_graph_tools {
bool savePoseGraphMsgToFile(const PoseGraph& graph,
                            const std::string& filename) {
  // Yulun: currently only save pose graph edges
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  file << "robot_from,key_from,robot_to,key_to,qx,qy,qz,qw,tx,ty,tz\n";
  for (size_t i = 0; i < graph.edges.size(); ++i) {
    PoseGraphEdge edge = graph.edges[i];
    geometry_msgs::Point position = edge.pose.position;
    geometry_msgs::Quaternion orientation = edge.pose.orientation;
    file << edge.robot_from << ",";
    file << edge.key_from << ",";
    file << edge.robot_to << ",";
    file << edge.key_to << ",";
    file << orientation.x << ",";
    file << orientation.y << ",";
    file << orientation.z << ",";
    file << orientation.w << ",";
    file << position.x << ",";
    file << position.y << ",";
    file << position.z << "\n";
  }

  return true;
}
}  // namespace pose_graph_tools