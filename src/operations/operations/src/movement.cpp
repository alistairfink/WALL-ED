#include "ros/ros.h"
#include "operations/movement.h"
#include "motor_driver/motor_driver.h"
#include "sensor_msgs/LaserScan.h"

float north;
float east;
float south;
float west;

void movement::turn(int direction, motor_abs::motor_driver* motor)
{
	float north_s = north;
	float east_s = east;
	float south_s = south;
	float west_s = west;
	if (direction == LEFT)
	{
		motor->set_speed(25, 25);
		while (north != west_s &&
			west != south_s &&
			south != east_s &&
			east != north_s);
	}
	else if (direction == RIGHT)
	{
		motor->set_speed(-25, -25);
		while (north != east_s &&
			east != south_s &&
			south != west_s &&
			west != north_s);
	}
	motor->set_speed(0,0);
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