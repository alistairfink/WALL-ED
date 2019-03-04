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

	achilles_mapping_service mapping_serv;

	ros::ServiceServer get_serv = n.advertiseService("get_course_map", &achilles_mapping_service::get_course_map_srv, &mapping_serv);

	ros::ServiceServer update_serv = n.advertiseService("update_course_map", &achilles_mapping_service::update_course_map_srv, &mapping_serv);
	

	ros::spin();

	return 0;
}