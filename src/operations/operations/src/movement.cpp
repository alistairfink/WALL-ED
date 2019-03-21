#include "ros/ros.h"
#include "operations/movement.h"
#include "motor_driver/motor_driver.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

static float north = 5;
static float east = 5;
static float south = 5;
static float west = 5;
static double yaw = 0;

void movement::turn(int direction, motor_abs::motor_driver* motor)
{
	float north_s = north;
	float east_s = east;
	float south_s = south;
	float west_s = west;

	float tol = 1;
	float straight = north_s + south_s;
	float side = east_s + west_s;
	if (direction == LEFT)
	{
		motor->set_speed(50, 50);
	}
	else if (direction == RIGHT)
	{
		motor->set_speed(-50, -50);
	}

	ros::Duration(0.5).sleep();
	while (std::abs((north + south) - side) <= tol &&
		std::abs((east + west) - straight) <= tol);
	motor->set_speed(0,0);
	// check pos?
}

void movement::straight(motor_abs::motor_driver* motor)
{
	float north_s = north;
	float south_s = south;
	float dist = 1;
	float tol = 0.01;
	motor->set_speed(100-movement::OFFSET,-100);
	ros::Duration(1).sleep();
	while (std::abs((north + dist) - north_s) > tol &&
		std::abs((south - dist) - south_s) > tol)
	{
		ros::spinOnce();
	}
	motor->set_speed(0,0);	
}

void movement::orientation(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Pose pose;
	tf::poseMsgToTF(msg->pose, pose);
	yaw = tf::getYaw(pose.getRotation());
}

void movement::get_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	north = msg->ranges[0];
	east = msg->ranges[90];
	south = msg->ranges[180];
	west = msg->ranges[270];

	ROS_INFO("FRONT: %f", north);
	ROS_INFO("RIGHT: %f", east);
	ROS_INFO("BACK: %f", south);
	ROS_INFO("LEFT: %f", west);
	ROS_INFO("------------------------");
}
