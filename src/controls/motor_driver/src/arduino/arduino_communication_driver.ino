#include <string.h>
#include "DualMC33926MotorShield.h"

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
void motor_speed(int motor1Speed, int motor2Speed)
{
    if (motor1Speed >= -400 && motor1Speed <= 400 && motor2Speed >= -400 && motor2Speed <= 400)
    {
        motor_shield.setM1Speed(motor1Speed);
        motor_shield.setM2Speed(motor2Speed);
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
 * Serial command expects "motor_1_speed,motor_2_speed,motor_1_speed*motor_2_speed,*"
 */
void loop() {
    static int motor_control[3];
    static char buff[30];
    int counter = 0;
    bool dirty = false;

    while (Serial.available() > 0)
    {
        buff[counter] = Serial.read();
        if (counter > 30 || buff[counter] == '*') 
        {
            buff[counter] = '\0';
            char* mc0 = strtok(buff,",");
            char* mc1 = strtok(NULL,",");
            char* mc2 = strtok(NULL,",");
            if (mc0 == NULL || mc1 == NULL || mc2 == NULL)
            {
                Serial.println("false");
                dirty = false;
                break;
            }

            motor_control[0]=atoi(mc0);
            motor_control[1]=atoi(mc1);
            motor_control[2]=atoi(mc2);
            dirty = true;
        }
        else
        {
            counter++;
        }
    }

    if (dirty && (motor_control[0]^motor_control[1]) == motor_control[2])
    {
        motor_speed(motor_control[0], motor_control[1]);
        Serial.println("true");
    }

    delay(2);
}
