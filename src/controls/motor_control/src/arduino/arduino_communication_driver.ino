#include <string.h>
#include "./DualMC33926MotorShield/DualMC33926MotorShield.h"

int led = 12;
void setup() {
    Serial.begin(115200);
    pinMode(led, OUTPUT);
}

void loop() {
    static int motor_control[3];
    static char buff[30];
    int counter = 0;
    bool dirty = false;

    // Reset the arduino when it's fucking itself
    /*
    digitalWrite(led, HIGH);
    delay(500);
    digitalWrite(led, LOW);
    delay(500);
    while (Serial.available() > 0) {}
    */

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
        for (int i = 0; i < motor_control[1]; i++)
        {
            digitalWrite(led, HIGH);
            delay(500);
            digitalWrite(led, LOW);
            delay(500);
        }
    }

    delay(100);
}
