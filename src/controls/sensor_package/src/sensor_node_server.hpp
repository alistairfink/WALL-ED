//
//  sensor_node_server.hpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#ifndef sensor_node_server_hpp
#define sensor_node_server_hpp

#include <stdio.h>
void get_imu_data();
int get_hall_effect_data();
void get_ultrasonic_data();
bool get_sensor_data(sensor_package::AddTwoInts::Request  &req, sensor_package::AddTwoInts::Response &res);
#endif /* sensor_node_server_hpp */
