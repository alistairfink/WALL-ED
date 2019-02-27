#include <iostream>
#include <stdint.h>
#include <string>
#include <sstream>
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

	// Reset the arduino when it's fucking itself
	/*while (1)
	{*/
	controls::motor_control mc;
	mc.motor = 1;
	mc.direction = 1;
	mc.speed = 12;

	std::ostringstream stm;
	stm << mc.motor << "," << mc.direction << "," << mc.speed << "*";

	std::string message = stm.str();
	std::cout << message << " " << i << std::endl;

	ser.write(message);
	i++;
	/*}*/

}
