#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <interactive_markers/interactive_marker_server.h>

#include <tf/transform_datatypes.h>

#include <pose_graph_tools/PoseGraph.h>

#include <unordered_map>

class Visualizer {
 public:
  Visualizer(const ros::NodeHandle& nh);

  void visualize();

 private:
  void PoseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  geometry_msgs::Point getPositionFromKey(int robot_id,
                                          long unsigned int key) const;

  void MakeMenuMarker(const tf::Pose& position, const std::string& id_number);

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

  typedef std::pair<int, long unsigned int> Node;  // robot id, key
  typedef std::pair<Node, Node> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::vector<Edge> rejected_loop_edges_;
  std::map<int, std::map<long unsigned int, tf::Pose> > keyed_poses_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer>
      interactive_mrkr_srvr_;
};

#endif