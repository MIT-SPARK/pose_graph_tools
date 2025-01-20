#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
//#include <interactive_markers/interactive_marker_server.h>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/rclcpp.hpp>

class Visualizer : rclcpp::Node {
 public:
  explicit Visualizer(const rclcpp::NodeOptions& options);

  void visualize();

  typedef std::pair<int, uint64_t> Node;  // robot id, key
  typedef std::pair<Node, Node> Edge;

 private:
  void callback(const pose_graph_tools_msgs::msg::PoseGraph& msg);

  geometry_msgs::msg::Point getPositionFromKey(int robot_id,
                                               uint64_t key) const;

 private:
  using Node = std::pair<int, uint64_t>;  // robot id, key
  using Edge = std::pair<Node, Node>;

  std::string frame_id_;

  // subscribers
  ros::Subscriber pose_graph_sub_;

  // publishers
  ros::Publisher<visualization_msgs::msg::MarkerArray> marker_array_pub_;

  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::map<int, std::map<uint64_t, geometry_msgs::msg::Pose>> keyed_poses_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_mrkr_srvr_;
};
