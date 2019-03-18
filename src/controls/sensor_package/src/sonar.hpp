//
//  sonar.hpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#ifndef DEF_SONAR
#define DEF_SONAR

class Sonar
{
public:
    Sonar();
    void init(int trigger, int echo);
    double distance(int timeout);
    
private:
    void recordPulseLength();
    int trigger;
    int echo;
    volatile long startTimeUsec;
    volatile long endTimeUsec;
    double distanceMeters;
    long travelTimeUsec;
    long now;
};

#endif