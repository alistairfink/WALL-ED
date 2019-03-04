#include <ros/ros.h>
#include "achilles_mapping.h"

/**
* main
* Main for achilles_mapping node
*
* @param argc arg count 0?
* @param argv arg vector (none expected)
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "achilles_mapping");
	
	achilles_mapping_service mapping_serv;

	ros::spin();

	return 0;
}