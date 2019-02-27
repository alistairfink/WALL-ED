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
	ros::NodeHandle n;

	// Subscribe to the /map for occupancy grids from Hector
	ros::Subscriber sub = n.subscribe("map", 1000, handle_map);
	ros::spin();

	return 0;
}