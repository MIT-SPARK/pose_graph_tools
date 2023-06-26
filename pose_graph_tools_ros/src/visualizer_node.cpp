#include <ros/console.h>
#include <ros/ros.h>

#include "pose_graph_tools_ros/visualizer.h"

int main(int argc, char *argv[]) {
  // Initiallize visualizer
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh("~");
  Visualizer viz(nh);
}
