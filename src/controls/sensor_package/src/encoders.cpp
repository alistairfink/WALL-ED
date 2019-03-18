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

#define  RoAPin    0
#define  RoBPin    1
#define  RoSPin    2

static volatile int globalCounter = 0 ;

unsigned char flag;
unsigned char Last_RoB_Status;
unsigned char Current_RoB_Status;

void rotaryDeal(void)
{
    Last_RoB_Status = digitalRead(RoBPin);
    
    while(!digitalRead(RoAPin)){
        Current_RoB_Status = digitalRead(RoBPin);
        flag = 1;
    }
    
    if(flag == 1){
        flag = 0;
        if((Last_RoB_Status == 0)&&(Current_RoB_Status == 1)){
            globalCounter ++;
            printf("globalCounter : %d\n",globalCounter);
        }
        if((Last_RoB_Status == 1)&&(Current_RoB_Status == 0)){
            globalCounter --;
            printf("globalCounter : %d\n",globalCounter);
        }
        
    }
}

void rotaryClear(void)
{
    if(digitalRead(RoSPin) == 0)
    {
        globalCounter = 0;
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
