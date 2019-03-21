#include "ros/ros.h"
#include "operations/movement.h"
#include "motor_driver/motor_driver.h"
#include "sensor_msgs/LaserScan.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

static float north = 5;
static float east = 5;
static float south = 5;
static float west = 5;
static double deg_yaw = 0;

const double PI  = 3.141592653589793238463;

void movement::turn(int direction, motor_abs::motor_driver* motor)
{
	double deg_yaw_s = deg_yaw;
	// degrees
	float tol = 5;
	double desired_angle = deg_yaw_s;

	if (direction == LEFT)
	{
		desired_angle = deg_yaw_s - 90;
		motor->set_speed(50 - movement::OFFSET, 50);
	} 
	else if (direction == RIGHT)
	{
		desired_angle = deg_yaw_s + 90;
		motor->set_speed(-50 - movement::OFFSET, -50);
	}

	while (std::abs(desired_angle - deg_yaw) > tol)
	{
		ros::spinOnce();
	}

	motor->set_speed(0,0);
}

void movement::straight(motor_abs::motor_driver* motor)
{
	float north_s = north;
	float south_s = south;
	// meters
	float dist = 1;
	float tol = 0.01;
	motor->set_speed(100-movement::OFFSET, -100);
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
	geometry_msgs::Quaternion orientation = msg->pose.orientation;
	double rad_yaw = tf::getYaw(orientation);
	deg_yaw = rad_yaw*180/PI;
}

void movement::get_lidar(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	north = msg->ranges[0];
	east = msg->ranges[90];
	south = msg->ranges[180];
	west = msg->ranges[270];
}
