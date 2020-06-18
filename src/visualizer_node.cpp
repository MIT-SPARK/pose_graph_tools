#include <pose_graph_tools/visualizer.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char *argv[]) {
  // Initiallize visualizer
  ros::init(argc, argv, "visualizer");
  ros::NodeHandle nh("~");
  Visualizer viz(nh);
}