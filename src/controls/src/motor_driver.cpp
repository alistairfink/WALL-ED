#include "ros/ros.h"
#include "controls/motor_control.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_drivers");
	
	ros::NodeHandle n;
	ros::Publisher motor_control_pub = n.advertise<controls::motor_control>("motor_driver/motor_control", 1);

	for (;;)
	{
		controls::motor_control msg;
		msg.forward = true;
		msg.speed = 1;
		motor_control_pub.publish(msg);
	}

}