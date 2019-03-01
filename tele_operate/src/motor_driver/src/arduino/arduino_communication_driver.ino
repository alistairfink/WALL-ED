#include <string.h>
#include "DualMC33926MotorShield.h"

const int MOTOR_1 = 1;
const int MOTOR_2 = 2;
DualMC33926MotorShield motor_shield;

/**
 * stop_if_fault
 * Stops execution if theres a fault
 */
void stop_if_fault()
{
    if (motor_shield.getFault())
    {
        while(1);
    }
}

/**
 * motor_speed
 * Changes the motor speed of the requested motor
 *
 * @param motor Which motor we're setting the speed of
 * @param speed What to set the speed to
 */
void motor_speed(int motor, int speed)
{
    if (speed >= -400 && speed <= 400)
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

/**
 * setup
 * Arduino setup. Opens serial connection and initializes motor shield stuff.
 */
void setup() {
    Serial.begin(115200);
    motor_shield.init();
}

/**
 * loop
 * Arduino loop function. Continuously polls for commands from pi
 */
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
