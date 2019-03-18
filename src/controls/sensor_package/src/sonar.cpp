//
//  sonar.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include "sonar.hpp"
#include <iostream>
#include "wiringPi.h"
using namespace std;

Sonar::Sonar(){}

void Sonar::init(int trigger, int echo)
{
    this->trigger=trigger;
    this->echo=echo;
    pinMode(trigger, OUTPUT);
    pinMode(echo, INPUT);
    digitalWrite(trigger, LOW);
    delay(500);
}

double Sonar::distance(int timeout)
{
    delay(10);
    
    digitalWrite(trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger, LOW);
    
    now=micros();
    
    while (digitalRead(echo) == LOW && micros()-now<timeout);
    recordPulseLength();
    
    travelTimeUsec = endTimeUsec - startTimeUsec;
    distanceMeters = 100*((travelTimeUsec/1000000.0)*340.29)/2;
    
    return distanceMeters;
}

void Sonar::recordPulseLength()
{
    startTimeUsec = micros();
    while ( digitalRead(echo) == HIGH );
    endTimeUsec = micros();
}

/*int main ()
{
    int trigger = 1
    int echo = 0;
    
    if (wiringPiSetup() == -1)
        return -1;
    
    Sonar sonar;
    sonar.init(trigger, echo);
    
    while(1){
        cout << "Distance is " << sonar.distance(30000) << " cm." << endl;
    }
}*/
