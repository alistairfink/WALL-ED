#include "ros/ros.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "wiringPi.h"
#include "encoder_handler.h"
#include "std_msgs/Int16.h"

#define  LoAPin    0 //where the pins connect
#define  LoBPin    1
#define  RoAPin    3
#define  RoBPin    4
#define  RoSPin    2

#define PUB_FREQ 10 // Publish freq in Hz


/**
* encoder_handler
* Constructor for the encoder_handler publisher. Continuously publishes encoder readings
*/
encoder_handler::encoder_handler()
{
    if(wiringPiSetup() < 0){
        ROS_WARN("Unable to setup wiringPi\n");
    }

    // Config pins
    pinMode(RoAPin, INPUT);
    pinMode(RoBPin, INPUT);

    // Create node handle
    ros::NodeHandle n;

    // Encoder publishers
    ros::Publisher encoder_left_publisher = n.advertise<std_msgs::Int16>("lwheel", 100);
    ros::Publisher encoder_right_publisher = n.advertise<std_msgs::Int16>("rwheel", 100);

    ros::Rate loop_rate(PUB_FREQ); 

    while (ros::ok())
    {

        std_msgs::Int16 lwheel_msg;
        std_msgs::Int16 rwheel_msg;

        lwheel_msg.data = this->motor_l();
        rwheel_msg.data = this->motor_r();

        encoder_left_publisher.publish(lwheel_msg);
        encoder_right_publisher.publish(rwheel_msg);

        loop_rate.sleep();
    }
}

/**
* ~encoder_handler
* Destructor for the encoder_handler publisher.
*/
encoder_handler::~encoder_handler()
{
}


/**
* motor_l
* Gets encoder reading for left motor
*
* @return signed 16bit int for encoder reading
*/
int16_t encoder_handler::motor_l(void)
{
    static int16_t globalCounterL = 0 ;
    this->Last_LoB_Status = digitalRead(LoBPin);
    
    while(!digitalRead(LoAPin)){
        this->Current_LoB_Status = digitalRead(LoBPin);
        this->flagL = 1;
    }
    
    if(this->flagL == 1){
        this->flagL = 0;
        if((this->Last_LoB_Status == 0)&&(this->Current_LoB_Status == 1)){
            globalCounterL ++;
            ROS_INFO("globalCounter : %d\n",globalCounterL);
        }
        if((this->Last_LoB_Status == 1)&&(this->Current_LoB_Status == 0)){
            globalCounterL --;
            ROS_INFO("globalCounter : %d\n",globalCounterL);
        }
        
    }

    return globalCounterL;
}

/**
* motor_r
* Gets encoder reading for right motor
*
* @return signed 16bit int for encoder reading
*/
int16_t encoder_handler::motor_r(void)
{
    static int16_t globalCounterR = 0 ;
    this->Last_RoB_Status = digitalRead(RoBPin);
    
    while(!digitalRead(RoAPin)){
        this->Current_RoB_Status = digitalRead(RoBPin);
        this->flagR = 1;
    }
    
    if(this->flagR == 1){
        this->flagR = 0;
        if((this->Last_RoB_Status == 0)&&(this->Current_RoB_Status == 1)){
            globalCounterR ++;
            ROS_INFO("globalCounter : %d\n",globalCounterR);
        }
        if((this->Last_RoB_Status == 1)&&(this->Current_RoB_Status == 0)){
            globalCounterR --;
            ROS_INFO("globalCounter : %d\n",globalCounterR);
        }
        
    }

    return globalCounterR;
}


/**
* rotary_clear
* Clears current encoder reading?
*/
void encoder_handler::rotary_clear(void)
{
    if(digitalRead(RoSPin) == 0)
    {
        int globalCounter = 0;
        ROS_INFO("globalCounter : %d\n",globalCounter);
        delay(1000);
    }
}
