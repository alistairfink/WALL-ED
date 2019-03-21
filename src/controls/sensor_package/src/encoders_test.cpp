//
//  sonar.cpp
//  sensor_node
//
//  Created by Abdulmalik Ibrahim on 2019-03-12.
//  Copyright © 2019 Abdulmalik Ibrahim. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "wiringPi.h"
#include "encoders.hpp"

#define  LoAPin    0 //where the pins connect
#define  LoBPin    1
#define  RoAPin    3
#define  RoBPin    4

#define  RoSPin    2

static volatile int globalCounterL = 0 ;
static volatile int globalCounterR = 0 ;

unsigned char flagL;
unsigned char Last_LoB_Status;
unsigned char Current_LoB_Status;

unsigned char flagR;
unsigned char Last_RoB_Status;
unsigned char Current_RoB_Status;

void waitForRead (void)
{
     while(!digitalRead(LoAPin) && !digitalRead(RoAPin))
     {
         if (digitalRead(LoAPin))
         {
             Current_LoB_Status = digitalRead(LoBPin);
             flagL = 1;
             motorL();
         }
         
         if(digitalRead(RoAPin))
         {
             Current_RoB_Status = digitalRead(RoBPin);
             flagR = 1;
             motorR();
         }
     }
}

void motorL(void)
{
    Last_LoB_Status = digitalRead(LoBPin);
    
    while(!digitalRead(LoAPin)){
        Current_LoB_Status = digitalRead(LoBPin);
        flagL = 1;
    }
    
    if(flagL == 1){
        flagL = 0;
        if((Last_LoB_Status == 0)&&(Current_LoB_Status == 1)){
            globalCounterL ++;
            printf("globalCounter : %d\n",globalCounterL);
        }
        if((Last_LoB_Status == 1)&&(Current_LoB_Status == 0)){
            globalCounterL --;
            printf("globalCounter : %d\n",globalCounterL);
        }
        
    }
}

void motorR(void)
{
    Last_RoB_Status = digitalRead(RoBPin);
    
    while(!digitalRead(RoAPin)){
        Current_RoB_Status = digitalRead(RoBPin);
        flagR = 1;
    }
    
    if(flagR == 1){
        flagR = 0;
        if((Last_RoB_Status == 0)&&(Current_RoB_Status == 1)){
            globalCounterR ++;
            printf("globalCounter : %d\n",globalCounterR);
        }
        if((Last_RoB_Status == 1)&&(Current_RoB_Status == 0)){
            globalCounterR --;
            printf("globalCounter : %d\n",globalCounterR);
        }
        
    }
}

void rotaryClear(void)
{
    if(digitalRead(RoSPin) == 0)
    {
        int globalCounter = 0;
        printf("globalCounter : %d\n",globalCounter);
        delay(1000);
    }
}

/*int main(void)
{
    if(wiringPiSetup() < 0){
        fprintf(stderr, "Unable to setup wiringPi:%s\n",strerror(errno));
        return 1;
    }
    
    pinMode(RoAPin, INPUT);
    pinMode(RoBPin, INPUT);
    //pinMode(RoSPin, INPUT);
    
    //pullUpDnControl(RoSPin, PUD_UP);
    
    while(1){
        rotaryDeal();
    }
    
    return 0;
}*/
