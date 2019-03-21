//
//  hall_effect.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include "hall_effect.hpp"
#include "wiringPiI2C.h"
#include "wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

bool GPIO_setup()
{
    if (wiringPiSetup () == -1) //checks to make sure pi is setup correctly. 
        return 0 ;
    
    int hallpin = 21;
    int hallpin2 = 22;
    int hallpin3 = 23;
    int hallpin4 = 24;
    int ledpin = 4;
    
    pinMode(hallpin, INPUT); //input is hallpin
    pinMode(hallpin2, INPUT);
    pinMode(hallpin3, INPUT);
    pinMode(hallpin4, INPUT);
    pinMode(ledpin, OUTPUT); //output is LED
    digitalWrite(ledpin, 0); //turn off LED at start    
    return 1;
}

int hall_output()
{
    int hallpin = 21;
    int hallpin2 = 22;
    int hallpin3 = 23;
    int hallpin4 = 24;
    int ledpin = 4;
    
   if(digitalRead(hallpin) == 0 || digitalRead(hallpin2) == 0 || digitalRead(hallpin3) == 0 || digitalRead(hallpin4) == 0 ) //0 input at hall_pin means magnet detected 
        return 1;
    else
        return 0;
}


