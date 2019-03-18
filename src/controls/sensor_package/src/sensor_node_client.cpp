//
//  sensor_node_client.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include "sensor_node_client.hpp"
#include "ros/ros.h"
#include "sensor_package/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_node_client");
    if (argc != 2)
    {
        ROS_INFO("usage: request_sensor_data");
        return 1;
    }
    
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sensor_package::AddTwoInts>("sensor_data");
    sensor_package::AddTwoInts srv;
    
    srv.request.input = atoll(argv[1]);
    if (client.call(srv))
    {
		if(srv.request.input == 1)
		{
			ROS_INFO("response data: %ld", (long int)srv.response.hall);	
		}
	
		else
			ROS_INFO("response data: %ld", (float)srv.response.sonar);
    }
    
    else
    {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
    
    return 0;
}
