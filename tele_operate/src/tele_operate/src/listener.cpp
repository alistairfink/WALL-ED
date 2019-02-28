#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "motor_driver.h"
#include "commands.h"

motor_abs::motor_driver md("/dev/ttyUSB0", 115200);

void chatterCallback(const std_msgs::Int64::ConstPtr& msg)
{
	ROS_INFO("%d", (int)(msg->data));
	switch (msg->data)
	{
		case forward:
			md.set_speed(motor_abs::MOTOR_1, 100);
			md.set_speed(motor_abs::MOTOR_2, 100);
			break;
		case back:
			md.set_speed(motor_abs::MOTOR_1, -100);
			md.set_speed(motor_abs::MOTOR_2, -100);
			break;
		case turn_left:
			md.set_speed(motor_abs::MOTOR_1, 100);
			md.set_speed(motor_abs::MOTOR_2, -100);
			break;
		case turn_right:
			md.set_speed(motor_abs::MOTOR_1, -100);
			md.set_speed(motor_abs::MOTOR_2, 100);
			break;
		default:
			md.set_speed(motor_abs::MOTOR_1, 0);
			md.set_speed(motor_abs::MOTOR_2, 0);
	}
}

int main(int argc, char **argv)
{
	if (!md.check_connection())
	{
		while(1);
	}

	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("tele_operate/command", 1, chatterCallback);
	ros::spin();
	return 0;
}