//
//  hall_effect.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include "hall_effect.hpp"
#include "src/WiringPi/wiringPi/wiringPiI2C.h"
#include "src/WiringPi/wiringPi/wiringPi.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

bool GPIO_setup()
{
    if (wiringPiSetup () == -1) //checks to make sure pi is setup correctly. 
        return 0 ;
    
    int hallpin = 16;
    int hallpin2 = 37;
    int ledpin = 15;
    
    pinMode(hallpin, INPUT); //input is hallpin
    pinMode(hallpin2, INPUT); 
    pinMode(ledpin, OUTPUT); //output is LED
    digitalWrite(ledpin, 0); //turn off LED at start    
    return 1;
}

int hall_output()
{
    int hallpin = 16;
    int hallpin2 = 37;
    int ledpin = 15;
    
   // if(digitalRead(hallpin) == 0) //0 input at hall_pin means magnet detected 
        return 1;
  //  else
      //  return 0;
}

/******python code**********/ 

/*int main (void)
 {
 //gpio.setmode(gpio.BOARD)
 //gpio.setwarnings(False)
 if (wiringPiSetup () == -1)
 return 1 ;
 
 int hallpin = 16;
 int hallpin2 = 37;
 int ledpin = 15;
 
 pinMode(hallpin, INPUT); //input is hallpin
 pinMode(hallpin2, INPUT);
 pinMode(ledpin, OUTPUT); //output is LED
 digitalWrite(ledpin, 0); //turn off LED at start
 
 while(1)
 {
 if(digitalRead(hallpin) == 0)
 {
 digitalWrite(ledpin, 1);
 std::cout <<"magnet detected"<<std::endl;
 }
 
 else
 {
 digitalWrite(ledpin, 0);
 std::cout <<"magnet not detected"<<std::endl;
 }
 }
 
 }*/


