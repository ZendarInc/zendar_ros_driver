#include <ros/ros.h>

#include "zendar_ros_driver/zendar_driver_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "zendar_driver_node");
  ros::NodeHandle node("~");
  std::string url;
  if (!node.getParam("url", url))
    ROS_FATAL("IP address of ZPU was not provided");
  zen::ZendarDriverNode converter(node, url, argc, argv);
  // loop until shut down
  converter.Run();
  return 0;
}
