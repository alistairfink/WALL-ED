#include "ros/ros.h"
#include "operations/movement.h"
#include "motor_driver/motor_driver.h"
#include "sensor_msgs/LaserScan.h"
//#include "operations/operations.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

static float north = 0;
static float east = 0;
static float south = 0;
static float west = 0;
static float sum_front = 0;
static float sum_left = 0;
static float sum_right = 0;
static double deg_yaw = 0;

const double PI  = 3.141592653589793238463;

void movement::turn(int direction, int direction_to_go, motor_abs::motor_driver* motor)
{
	ros::spinOnce();
	double deg_yaw_s = deg_yaw;
	// degrees
	double tol = 30;
	double desired_angle = 0;

	if (direction_to_go == movement::DIR_WEST)
	{
		desired_angle = 0;
	}
	else if (direction_to_go == movement::DIR_NORTH)
	{
		desired_angle = -90;
	}
	else if (direction_to_go == movement::DIR_EAST)
	{
		desired_angle = 180;
	}
	else if (direction_to_go == movement::DIR_SOUTH)
	{
		desired_angle = 90;
	}
	ROS_INFO("Desired Angle: %i", desired_angle);
	if (direction == LEFT)
	{
		motor->set_speed(50 - movement::OFFSET, 50);
	}
	else if (direction == RIGHT)
	{
		motor->set_speed(-50 - movement::OFFSET, -50);
	}

	while (std::abs(desired_angle - deg_yaw) > tol)
	{
		ros::spinOnce();
	}
	
	motor->set_speed(0,0);
}

void movement::straight(int speed, float dist, motor_abs::motor_driver* motor)
{
	ros::spinOnce();
	float north_s = north;
	float south_s = south;
	// meters
	float tol = 0.01;
	motor->set_speed(speed-movement::OFFSET, -speed);
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

float movement::roll_up(float dist_from_target, int speed, motor_abs::motor_driver* motor)
{
	ros::spinOnce();
	float starting = north;
	float tol = 0.01;
	motor->set_speed(speed-movement::OFFSET, -speed);
	while (std::abs(north - dist_from_target) > tol)
	{
		ros::spinOnce();
	}

	motor->set_speed(0,0);
	return starting;
}
