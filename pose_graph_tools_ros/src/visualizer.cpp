#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pose_graph_tools_ros {

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Quaternion;
using pose_graph_tools_msgs::msg::PoseGraph;
using pose_graph_tools_msgs::msg::PoseGraphEdge;
using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

using KeyedPoses = std::map<int, std::map<uint64_t, Pose>>;

class Visualizer : public rclcpp::Node {
 public:
  using Node = std::pair<int, uint64_t>;  // robot id, key
  using Edge = std::pair<Node, Node>;

  explicit Visualizer(const rclcpp::NodeOptions& options);

  void visualize();

 private:
  void callback(const PoseGraph& msg);

 private:
  // state
  std_msgs::msg::Header last_header_;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::vector<Edge> other_edges_;
  KeyedPoses keyed_poses_;
  // params
  bool show_heading_;
  double odom_scale_;
  double loop_closure_scale_;
  double rejected_loop_closure_scale_;
  double other_edge_scale_;
  double node_scale_;
  double text_scale_;
  // ros infrastructure
  rclcpp::Subscription<PoseGraph>::SharedPtr sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

using VisualizerEdges = std::vector<Visualizer::Edge>;

namespace {

std_msgs::msg::ColorRGBA makeColor(int robot_id) {
  // TODO(Yun) currently the below color formula
  // means that only support up to 5 robots
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(robot_id) / 5;
  color.g = 1 - static_cast<float>(robot_id) / 5;
  color.b = 0.0;
  color.a = 0.8;
  return color;
}

Point positionFromKey(const KeyedPoses& keyed_poses,
                      int robot_id,
                      uint64_t key) {
  return keyed_poses.at(robot_id).at(key).position;
}

Quaternion orientationFromKey(const KeyedPoses& keyed_poses,
                              int robot_id,
                              uint64_t key) {
  return keyed_poses.at(robot_id).at(key).orientation;
}

void makeMenuMarker(interactive_markers::InteractiveMarkerServer& server,
                    const std_msgs::msg::Header& header,
                    const Pose& position,
                    const std::string& id_number) {
  interactive_markers::MenuHandler menu_handler;

  InteractiveMarker int_marker;
  int_marker.header = header;
  int_marker.scale = 1.0;
  int_marker.pose = position;
  int_marker.name = id_number;

  Marker marker;
  marker.type = Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;
  marker.pose.orientation.w = 1.0;

  InteractiveMarkerControl control;
  control.interaction_mode = InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back(marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  server.insert(int_marker);
  menu_handler.apply(server, int_marker.name);
}

Marker build_edge_marker(
    const KeyedPoses& keyed_poses,
    const VisualizerEdges& edges,
    const std::string& ns,
    const std::function<std_msgs::msg::ColorRGBA(size_t)>& cmap,
    double scale) {
  Marker marker;
  marker.ns = ns;
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.type = Marker::LINE_LIST;
  marker.scale.x = scale;
  marker.pose.orientation.w = 1.0;

  for (const auto& [from, to] : edges) {
    const auto& [from_robot, from_key] = from;
    const auto& [to_robot, to_key] = to;
    marker.points.push_back(positionFromKey(keyed_poses, from_robot, from_key));
    marker.points.push_back(positionFromKey(keyed_poses, to_robot, to_key));
    marker.colors.push_back(cmap(from_robot));
    marker.colors.push_back(cmap(to_robot));
  }

  return marker;
}

std::vector<Marker> build_heading_markers(const KeyedPoses& keyed_poses,
                                          const VisualizerEdges& odom_edges) {
  std::vector<Marker> arrows;

  int id = 0;
  for (size_t ii = 0; ii < odom_edges.size(); ++ii) {
    Marker marker;
    marker.ns = "heading";
    marker.id = id++;
    marker.action = Marker::ADD;
    marker.type = Marker::ARROW;
    int robot_id = odom_edges[ii].first.first;
    const auto key = odom_edges[ii].first.second;

    marker.color = makeColor(robot_id);
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.orientation.w = 1.0;

    auto pos = positionFromKey(keyed_poses, robot_id, key);
    auto quat = orientationFromKey(keyed_poses, robot_id, key);
    marker.pose.position = pos;
    marker.pose.orientation = quat;
    arrows.push_back(marker);
  }

  return arrows;
}

std::vector<Marker> build_node_ids(const KeyedPoses& keyed_poses,
                                   double scale) {
  // Publish loop closure edges.
  // Publish node IDs in the pose graph.

  std::vector<Marker> m_ids;
  int id_base = 100;
  for (const auto& robot : keyed_poses) {
    for (const auto& [key, pose] : robot.second) {
      auto& marker = m_ids.emplace_back();
      marker.ns = "NodeIds";
      marker.action = Marker::ADD;
      marker.type = Marker::TEXT_VIEW_FACING;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.2;
      marker.color.a = 0.8;
      // Only Scale z is used - height of capital A in the text
      marker.scale.z = scale;
      marker.pose.orientation.w = 1.0;
      marker.pose = pose;
      // Display text for the node
      std::string robot_id = std::to_string(key);
      marker.text = robot_id;
      marker.id = id_base + key;
    }
  }

  return m_ids;
}

Marker build_keyframes(const KeyedPoses& keyed_poses, double scale) {
  // Publish keyframe nodes in the pose graph.
  Marker marker;
  marker.ns = "keyframes";
  marker.id = 0;
  marker.action = Marker::ADD;
  marker.type = Marker::SPHERE_LIST;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.3;
  marker.color.a = 0.8;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  marker.pose.orientation.w = 1.0;

  for (const auto& robot : keyed_poses) {
    for (const auto& keyedPose : robot.second) {
      marker.points.push_back(
          positionFromKey(keyed_poses, robot.first, keyedPose.first));
    }
  }
  return marker;
}

}  // namespace

Visualizer::Visualizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pose_graph_visualizer", options) {
  RCLCPP_INFO(get_logger(), "Initializing pose graph visualizer");

  // start subscribers
  sub_ = create_subscription<PoseGraph>(
      "graph", 10, [this](const PoseGraph& msg) -> void { callback(msg); });
  pub_ = create_publisher<MarkerArray>("pose_graph_markers", 10);

  declare_parameter("use_server", false);
  auto use_server = get_parameter("use_server").as_bool();
  declare_parameter("show_heading", false);
  get_parameter("show_heading", show_heading_);
  declare_parameter("odom_scale", 0.02);
  get_parameter("odom_scale", odom_scale_);
  declare_parameter("loop_closure_scale", 0.02);
  get_parameter("loop_closure_scale", loop_closure_scale_);
  declare_parameter("rejected_loop_closure_scale", 0.02);
  get_parameter("rejected_loop_closure_scale", rejected_loop_closure_scale_);
  declare_parameter("other_edge_scale", 0.02);
  get_parameter("other_edge_scale", other_edge_scale_);
  declare_parameter("node_scale", 0.15);
  get_parameter("node_scale", node_scale_);
  declare_parameter("text_scale", 0.1);
  get_parameter("text_scale", text_scale_);

  if (use_server) {
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_node", this);
  }
}

void Visualizer::callback(const PoseGraph& msg) {
  // iterate through nodes in pose graph
  keyed_poses_.clear();
  for (const auto& msg_node : msg.nodes) {
    // Fill pose nodes (representing the robot position)
    keyed_poses_[msg_node.robot_id][msg_node.key] = msg_node.pose;
  }

  // update frame id
  last_header_ = msg.header;

  odometry_edges_.clear();
  loop_edges_.clear();
  rejected_loop_edges_.clear();
  other_edges_.clear();
  // iterate through edges in pose graph
  for (const auto& msg_edge : msg.edges) {
    Node from = std::make_pair(msg_edge.robot_from, msg_edge.key_from);
    Node to = std::make_pair(msg_edge.robot_to, msg_edge.key_to);
    if (msg_edge.type == PoseGraphEdge::ODOM) {
      // initialize first seen robot id
      odometry_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type == PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type == PoseGraphEdge::REJECTED_LOOPCLOSE) {
      rejected_loop_edges_.emplace_back(std::make_pair(from, to));
    } else if (msg_edge.type == PoseGraphEdge::MESH) {
      other_edges_.emplace_back(std::make_pair(from, to));
    }
  }

  visualize();
}

void Visualizer::visualize() {
  if (!pub_->get_subscription_count()) {
    return;
  }

  std_msgs::msg::ColorRGBA lc_color;
  lc_color.r = 0.0;
  lc_color.g = 0.2;
  lc_color.b = 1.0;
  lc_color.a = 0.8;

  std_msgs::msg::ColorRGBA reject_color;
  reject_color.r = 0.5;
  reject_color.g = 0.5;
  reject_color.b = 0.5;
  reject_color.a = 0.7;

  MarkerArray msg;

  // Odometry Edges
  msg.markers.push_back(build_edge_marker(
      keyed_poses_, odometry_edges_, "odom_edges", &makeColor, odom_scale_));

  // Loop Closures
  msg.markers.push_back(build_edge_marker(
      keyed_poses_,
      loop_edges_,
      "loop_closure_edges",
      [&](size_t) { return lc_color; },
      loop_closure_scale_));

  // Rejected Loop Closures
  msg.markers.push_back(build_edge_marker(
      keyed_poses_,
      rejected_loop_edges_,
      "rejected_loop_closure_edges",
      [&](size_t) { return reject_color; },
      rejected_loop_closure_scale_));

  // Any Other Edge
  msg.markers.push_back(build_edge_marker(keyed_poses_,
                                          other_edges_,
                                          "other_edges",
                                          &makeColor,
                                          other_edge_scale_));

  // Node IDs
  auto m_node_ids = build_node_ids(keyed_poses_, text_scale_);
  msg.markers.insert(msg.markers.end(), m_node_ids.begin(), m_node_ids.end());

  // Keyframes
  auto m_keyframes = build_keyframes(keyed_poses_, node_scale_);
  msg.markers.push_back(m_keyframes);

  // Heading (orientation) Edges
  if (show_heading_) {
    auto m_headings = build_heading_markers(keyed_poses_, odometry_edges_);
    msg.markers.insert(msg.markers.end(), m_headings.begin(), m_headings.end());
  }

  for (auto& marker : msg.markers) {
    marker.header = last_header_;
  }

  // Publish it all!
  pub_->publish(msg);

  if (!server_) {
    return;
  }

  // Interactive Markers
  for (const auto& robot : keyed_poses_) {
    for (const auto& [key, pose] : robot.second) {
      // Display text for the node
      std::string robot_id = std::to_string(key);
      makeMenuMarker(*server_, last_header_, pose, robot_id);
    }
  }

  server_->applyChanges();
}

}  // namespace pose_graph_tools_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pose_graph_tools_ros::Visualizer)
