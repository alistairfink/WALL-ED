#include "ros/ros.h"
#include "std_msgs/Int64.h"

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
	ROS_INFO("%d", (int)(msg->data));
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tele_operate/command", 1, chatterCallback);
	ros::spin();
	return 0;
}