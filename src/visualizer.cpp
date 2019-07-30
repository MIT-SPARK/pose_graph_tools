#include <pose_graph_tools/visualizer.h>

Visualizer::Visualizer() {
	ROS_INFO("Initializing pose graph visualizer");

	// get parameters
	nh_.getParam("frame_id", frame_id_);

	// start subscribers
  pose_graph_sub_ = nh_.subscribe<pose_graph_tools::PoseGraph>(
      "graph", 10, &Visualizer::PoseGraphCallback, this);

  // start publishers
  odometry_edge_pub_ = nh_.advertise<visualization_msgs::Marker>(
  		"odometry_edges", 10, false);
  loop_edge_pub_ = nh_.advertise<visualization_msgs::Marker>(
  		"loop_edges", 10, false);
  graph_node_pub_ = nh_.advertise<visualization_msgs::Marker>(
  		"graph_nodes", 10, false);
  graph_node_id_pub_ = nh_.advertise<visualization_msgs::Marker>(
  		"graph_nodes_ids", 10, false);
}

void Visualizer::PoseGraphCallback(
		const pose_graph_tools::PoseGraph::ConstPtr& msg) {
	// iterate through nodes in pose graph
	for (const pose_graph_tools::PoseGraphNode &msg_node : msg->nodes) {
    // tf::Pose pose;
    // tf::poseMsgToTF(msg_node.pose, pose);

    // // Fill pose nodes (representing the robot position)
    // keyed_poses_[msg_node.key] = pose;

    // keyed_stamps_.insert(std::pair<long unsigned int, ros::Time>(
    //     msg_node.key, msg_node.header.stamp));
    // stamps_keyed_.insert(std::pair<double, long unsigned int>(
    //     msg_node.header.stamp.toSec(), msg_node.key));
  }

  // iterate through edges in pose graph
  for (const pose_graph_tools::PoseGraphEdge &msg_edge : msg->edges) {
    if (msg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
      odometry_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
    } else if (msg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      loop_edges_.emplace_back(
          std::make_pair(msg_edge.key_from, msg_edge.key_to));
    }
  }
}

geometry_msgs::Point Visualizer::getPositionFromKey(
		long unsigned int key) const {
	tf::Vector3 v = keyed_poses_.at(key).getOrigin();
	geometry_msgs::Point p;
  p.x = v.x();
  p.y = v.y();
  p.z = v.z();
  return p;
}

void Visualizer::visualize() {
	// Publish odometry edges.
  if (odometry_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      const auto key1 = odometry_edges_[ii].first;
      const auto key2 = odometry_edges_[ii].second;

      m.points.push_back(getPositionFromKey(key1));
      m.points.push_back(getPositionFromKey(key2));
    }
    odometry_edge_pub_.publish(m);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.0;
    m.color.g = 0.2;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
      const auto key1 = loop_edges_[ii].first;
      const auto key2 = loop_edges_[ii].second;

      m.points.push_back(getPositionFromKey(key1));
      m.points.push_back(getPositionFromKey(key2));
    }
    loop_edge_pub_.publish(m);
  }

  // Publish node IDs in the pose graph.
  if (graph_node_id_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;

    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    m.scale.z = 0.02; // Only Scale z is used - height of capital A in the text

    int id_base = 100;
    int counter = 0;
    for (const auto &keyedPose : keyed_poses_) {
      tf::poseTFToMsg(keyedPose.second, m.pose);
      // Display text for the node
      m.text = std::to_string(keyedPose.first);
      m.id = id_base + keyedPose.first;
      graph_node_id_pub_.publish(m);
    }
  }

  // Publish keyframe nodes in the pose graph.
  if (graph_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id_;
    m.ns = frame_id_;
    m.id = 4;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.3;
    m.color.a = 0.8;
    m.scale.x = 0.25;
    m.scale.y = 0.25;
    m.scale.z = 0.25;

    for (const auto &keyedPose : keyed_poses_) {
      m.points.push_back(getPositionFromKey(keyedPose.first));
    }
    graph_node_pub_.publish(m);
  }
}