#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_datatypes.h>

#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>

#include <unordered_map>
#include <map>
#include <vector>

class Visualizer {
public:
	Visualizer();

	void visualize();

private:
	void PoseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

	geometry_msgs::Point getPositionFromKey(long unsigned int key) const;

private:
	ros::NodeHandle nh_;
	std::string frame_id_;

	// subscribers
	ros::Subscriber pose_graph_sub_;

	// publishers
	ros::Publisher odometry_edge_pub_;
  ros::Publisher loop_edge_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher graph_node_id_pub_;

	typedef std::pair<long unsigned int, long unsigned int> Edge;
	std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;
  std::unordered_map<long unsigned int, tf::Pose> keyed_poses_;
};

#endif