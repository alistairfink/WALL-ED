#include <ros/ros.h>
#include "encoder_handler.h"

/**
* main
* Main for encoder_handler node
*
* @param argc arg count 0?
* @param argv arg vector (none expected)
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "encoder_handler");
	
	encoder_handler encoder_publisher;

	return 0;
}