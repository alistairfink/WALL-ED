#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "ros/ros.h"
#include "motor_driver/motor_driver.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"

namespace movement {
	const int LEFT = 0;
	const int RIGHT = 1;
	const int OFFSET = 6;

	void turn(int direction, motor_abs::motor_driver* motor);
	void straight(motor_abs::motor_driver* motor);
	void orientation(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void get_lidar(const sensor_msgs::LaserScan::ConstPtr& msg);
}

#endif
