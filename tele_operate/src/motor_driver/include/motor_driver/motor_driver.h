#ifndef MOTOR_ABS_H_
#define MOTOR_ABS_H_

#include "ros/ros.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"

namespace motor_abs {
	/**
	 * Motor Constants
	 * Useful for changing speeds.
	 */
	const int16_t MAX_FORWARD = 400;
	const int16_t MAX_BACKWARD = -400;

	serial::Serial* connection;
	std::string format(int16_t motor_1_speed, int16_t motor_2_speed);

	//motor_driver(std::string port, uint32_t baud);
	bool check_connection();	
	void set_speed(int16_t motor_1_speed, int16_t motor_2_speed);
	void translate_vel(const geometry_msgs::Twist::ConstPtr& msg);
}

#endif