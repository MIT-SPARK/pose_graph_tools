#include "pose_graph_tools_ros/utils.h"

#include <fstream>

#include <ros/console.h>

namespace pose_graph_tools {

bool savePoseGraphEdgesToFile(const pose_graph_tools_msgs::PoseGraph &graph,
                              const std::string &filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  file << "robot_from,key_from,robot_to,key_to,qx,qy,qz,qw,tx,ty,tz\n";
  for (size_t i = 0; i < graph.edges.size(); ++i) {
    pose_graph_tools_msgs::PoseGraphEdge edge = graph.edges[i];
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

pose_graph_tools_msgs::PoseGraph filterDuplicateEdges(
    const pose_graph_tools_msgs::PoseGraph &graph_in) {
  pose_graph_tools_msgs::PoseGraph graph_out;

  graph_out.nodes = graph_in.nodes;

  for (size_t i = 0; i < graph_in.edges.size(); ++i) {
    pose_graph_tools_msgs::PoseGraphEdge edge_in = graph_in.edges[i];
    bool skip = false;
    for (size_t j = 0; j < graph_out.edges.size(); ++j) {
      pose_graph_tools_msgs::PoseGraphEdge edge = graph_out.edges[j];
      if (edge.robot_from == edge_in.robot_from && edge.robot_to == edge_in.robot_to &&
          edge.key_from == edge_in.key_from && edge.key_to == edge_in.key_to) {
        skip = true;
        break;
      }
    }
    if (!skip) {
      graph_out.edges.push_back(edge_in);
    }
  }

  unsigned int num_edges_in = graph_in.edges.size();
  unsigned int num_edges_out = graph_out.edges.size();
  printf("Detected and removed %u duplicate edges from pose graph.\n",
         num_edges_in - num_edges_out);

  return graph_out;
}

}  // namespace pose_graph_tools
