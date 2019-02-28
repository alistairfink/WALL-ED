#include <string.h>
#include "DualMC33926MotorShield.h"

const int MOTOR_1 = 1;
const int MOTOR_2 = 2;
DualMC33926MotorShield motor_shield;

void stop_if_fault()
{
    if (motor_shield.getFault())
    {
        while(1);
    }
}

void motor_speed(int motor, int speed)
{
    if (speed > -400 && speed < 400)
    {
        if (motor == MOTOR_1)
        {
            motor_shield.setM1Speed(speed);
        }
        else if (motor == MOTOR_2)
        {
            motor_shield.setM2Speed(speed);
        }
        stop_if_fault();
    }    
}

void setup() {
    Serial.begin(115200);
    motor_shield.init();
}

void loop() {
    static int motor_control[2];
    static char buff[30];
    int counter = 0;
    bool dirty = false;

    while (Serial.available() > 0)
    {
        buff[counter] = Serial.read();
        if (counter > 30 || buff[counter] == '*') 
        {
            buff[counter] = '\0';
            motor_control[0]=atoi(strtok(buff,","));
            motor_control[1]=atoi(strtok(NULL,","));
            dirty = true;
        }
        else
        {
            counter++;
        }
    }

    if (dirty)
    {
        motor_speed(motor_control[0], motor_control[1]);
    }

    delay(2);
}
