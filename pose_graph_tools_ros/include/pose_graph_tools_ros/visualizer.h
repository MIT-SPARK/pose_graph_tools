#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <ros/ros.h>

class Visualizer {
 public:
  explicit Visualizer(const ros::NodeHandle& nh);

  void visualize();

 private:
  void PoseGraphCallback(const pose_graph_tools_msgs::PoseGraph::ConstPtr& msg);

  geometry_msgs::Point getPositionFromKey(int robot_id, uint64_t key) const;

  void MakeMenuMarker(const geometry_msgs::Pose& position,
                      const std::string& id_number);

 private:
  std::string frame_id_;

  // subscribers
  ros::Subscriber pose_graph_sub_;

  // publishers
  ros::Publisher odometry_edge_pub_;
  ros::Publisher loop_edge_pub_;
  ros::Publisher rejected_loop_edge_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher graph_node_id_pub_;

  typedef std::pair<int, uint64_t> Node;  // robot id, key
  typedef std::pair<Node, Node> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::map<int, std::map<uint64_t, geometry_msgs::Pose>> keyed_poses_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_mrkr_srvr_;
};
