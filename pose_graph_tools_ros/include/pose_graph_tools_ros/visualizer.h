#pragma once

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Visualizer : rclcpp::Node {
 public:
  using Node = std::pair<int, uint64_t>;  // robot id, key
  using Edge = std::pair<Node, Node>;

  explicit Visualizer(const rclcpp::NodeOptions& options);

  void visualize();

 private:
  void callback(const pose_graph_tools_msgs::msg::PoseGraph& msg);

 private:
  // state
  std::string frame_id_;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::map<int, std::map<uint64_t, geometry_msgs::msg::Pose>> keyed_poses_;
  // ros infrastructure
  rclcpp::Subscription<pose_graph_tools_msgs::msg::PoseGraph>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};
