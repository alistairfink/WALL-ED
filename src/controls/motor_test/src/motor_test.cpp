#include "ros/ros.h"
#include "motor_driver/motor_driver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_test");

	motor_abs::motor_driver md = motor_abs::motor_driver("/dev/ttyUSB0", 115200);

	if (!md.check_connection())
	{
		while(1);
	}

	ros::Duration(2).sleep();
	md.set_speed(-50, 50);
	ros::Duration(2).sleep();
	md.set_speed(50, -50);
	ros::Duration(2).sleep();
	md.set_speed(50, 50);
	ros::Duration(2).sleep();
	md.set_speed(-50, -50);
	ros::Duration(2).sleep();
	md.set_speed(0, 0);

	return 0;	
}