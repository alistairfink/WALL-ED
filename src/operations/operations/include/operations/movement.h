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
	const float TILE_DIST = 0.3;
	const int NOMINAL = 150;
	const int DIR_NORTH = 0;
	const int DIR_EAST = 1;
	const int DIR_SOUTH = 2;
	const int DIR_WEST = 3;

	void turn(int direction, int direction_to_go, motor_abs::motor_driver* motor);
	void straight(int speed, float dist, motor_abs::motor_driver* motor);
	void orientation(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void get_lidar(const sensor_msgs::LaserScan::ConstPtr& msg);
	float roll_up(float dist_from_target, int speed, motor_abs::motor_driver* motor);
}

#endif
