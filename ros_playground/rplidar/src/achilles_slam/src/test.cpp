#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"


void handle_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("Frame is [%d]", msg->header.seq);
  ROS_INFO("Resolution is [%f]", msg->info.resolution);
  ROS_INFO("Width is [%d]", msg->info.width);
  ROS_INFO("Height is [%d]\n", msg->info.height);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "achilles_test");
  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("map", 1000, handle_map);
  ros::spin();

  return 0;
}