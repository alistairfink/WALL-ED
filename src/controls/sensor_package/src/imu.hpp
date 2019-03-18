//
//  imu.hpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#ifndef imu_hpp
#define imu_hpp

#include <stdio.h>
void MPU6050_Init();

short read_raw_data(int addr);

void ms_delay(int val);

int *poll_imu();

#endif /* imu_hpp */
