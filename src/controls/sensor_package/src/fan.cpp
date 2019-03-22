#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <iostream>
#include "fan.hpp"
using namespace std;

int turnOnFan(){
    
    pinMode (5, OUTPUT);
    digitalWrite(5, HIGH);
    
    return 1;
}

int turnOffFan(){
    
    pinMode (5, OUTPUT);
    digitalWrite(5, LOW);
    
    return 1;
}

/*int main (void)
{
  wiringPiSetup();
  pinMode (5, OUTPUT);
  digitalWrite(5, LOW);
  bool a;

while (1)
{
        cin >> a;
        if (a)
        {
        cout<<"TEST HIGH"<<endl;
        digitalWrite(5, HIGH);
        }

        if(!a)
        {
        cout <<"TEST LOW"<<endl;
        digitalWrite(5, LOW);
        }
}  

return 0;

}*/
