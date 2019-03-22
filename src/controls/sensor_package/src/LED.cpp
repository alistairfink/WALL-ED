#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <iostream>
#include "LED.hpp"
using namespace std;

int turnOnLED(){
    
    pinMode (4, OUTPUT);
    digitalWrite(4, HIGH);
    
    return 1;
}

int turnOffLED(){
    
    pinMode (4, OUTPUT);
    digitalWrite(4, LOW);
    
    return 1;
}
