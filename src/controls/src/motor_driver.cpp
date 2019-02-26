#include <iostream>
#include <stdint.h>
#include "ros/ros.h"
#include "controls/motor_control.h"
#include "serial/serial.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_drivers");
	
	ros::NodeHandle n;
	ros::Publisher motor_control_pub = n.advertise<controls::motor_control>("motor_driver/motor_control", 1);

	serial::Serial ser("/dev/ttyUSB0",115200,serial::Timeout::simpleTimeout(1000));
	if (ser.isOpen())
	{
		std::cout << "Connected" << std::endl;
	}

	controls::motor_control ms;
	ms.forward = true;
	ms.speed = 1;
	motor_control_pub.publish(ms);
	std::vector<unsigned char> msg_toSend (2, 0);
	msg_toSend.insert(msg_toSend.begin() + 1, (unsigned char)(ms.speed));//(int)(ms.forward)));
	msg_toSend.insert(msg_toSend.begin() + 1, (unsigned char)(ms.speed));
//	std::cout << "Stuff: " << msg_toSend[0] << " " << msg_toSend[1];
	ser.write(msg_toSend);

	for (;;)
	{
		controls::motor_control msg;
		msg.forward = true;
		msg.speed = 1;
		motor_control_pub.publish(msg);
	}

}
