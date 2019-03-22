//
//  sensor_node_server.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include "sensor_package/AddTwoInts.h"
#include "sensor_node_server.hpp"
#include "imu.hpp"
#include "sonar.hpp"
#include "hall_effect.hpp"
#include "encoders.hpp"
#include "ros/ros.h"
#include "fan.hpp"

//service node
bool get_sensor_data(sensor_package::AddTwoInts::Request  &req, sensor_package::AddTwoInts::Response &res)
{
    if (req.input == 1)
    {
		res.hall = hall_output();
	}
    else if(req.input == 2)
    {
        res.fanOn = turnOnFan();
    }
    else if (req.input == 3)
    {
        res.fanOff = turnOffFan();
    }
    
    ROS_INFO("request: sensor=%ld", (long int)req.input);
    ROS_INFO("sending back response: [%ld]", (long int)res.hall, (long int)res.fanOn, (long int)res.fanOff );
    return true;
    
}

int main(int argc, char **argv)
{
   // MPU6050_Init();
    
    if(!(GPIO_setup()))
        return 1;
    
    ros::init(argc, argv, "sensor_node_server");
    ros::NodeHandle n;
    
    ros::ServiceServer service = n.advertiseService("sensor_data", get_sensor_data);
    ROS_INFO("Ready to get sensor data.");
    ros::spin();
    return 0;
}
