#include <string>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "motor_driver/motor_driver.h"
#include "geometry_msgs/Twist.h"

/**
 * format
 * Formats string into format to send across serial
 *
 * @param motor Which motor we're setting the speed of
 * @param speed What to set the speed to
 * @return Formatted string with two speeds xor'd for crcish thing
 */
std::string motor_abs::format(int16_t motor_1_speed, int16_t motor_2_speed)
{
	std::ostringstream stm;
	stm << motor_1_speed << "," << motor_2_speed << "," << (motor_1_speed^motor_2_speed) <<"*";
	return stm.str();
}

/**
 * check_connection
 * Checks if serial connection is open
 * 
 * @return Returns if connection is open or not
 */
bool motor_abs::check_connection()
{
	return connection->isOpen(); 
}

/**
 * set_speed
 * Sets the speed of a motor
 *
 * @param motor Which motor we're setting the speed of
 * @param speed What to set the speed to
 */
void motor_abs::set_speed(int16_t motor_1_speed, int16_t motor_2_speed)
{
	if (motor_1_speed >= MAX_BACKWARD && motor_1_speed <= MAX_FORWARD && motor_2_speed >= MAX_BACKWARD && motor_2_speed <= MAX_FORWARD)
	{
		std::string message = format(motor_1_speed, motor_2_speed);
		std::string result = "";
		while (result != "true")
		{
			connection->write(message);
			result = connection->readline();
			result = result.substr(0, result.size()-2);
		}
	}
}

void motor_abs::translate_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
	int target_vel = msg->linear.x;
	int target_angular_vel = msg->angular.z;
	int radius = 1;
	int wheel_distance = 1;

	int vel_r = (2*target_vel + target_angular_vel*wheel_distance) / (2);
    int vel_l = (2*target_vel - target_angular_vel*wheel_distance) / (2);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_driver");
  	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 1, motor_abs::translate_vel);
	
	motor_abs::connection = new serial::Serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
	while (!motor_abs::check_connection());

	ros::spin();
	return 0;
}