#include "ros/ros.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include "wiringPi.h"
#include "encoder_handler.h"
#include "std_msgs/Int16.h"

#define  LoAPin    0 //where the pins connect
#define  LoBPin    1
#define  RoAPin    2
#define  RoBPin    3
#define  RoSPin    24

#define PUB_FREQ 50 // Publish freq in Hz


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
	pinMode(LoAPin, INPUT);
	pinMode(LoBPin, INPUT);

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
	static unsigned char flagL;
	static unsigned char Last_LoB_Status;
	static unsigned char Current_LoB_Status;

	static int16_t globalCounterL = 0 ;
	Last_LoB_Status = digitalRead(LoBPin);

	volatile int temp_read = digitalRead(LoAPin);
	
	if(!temp_read){
		Current_LoB_Status = digitalRead(LoBPin);
		flagL = 1;
		return globalCounterL;
	}
	
	if(flagL == 1){
		flagL = 0;
		if((Last_LoB_Status == 0)&&(Current_LoB_Status == 1)){
			globalCounterL ++;
			ROS_DEBUG("globalCounter : %d\n",globalCounterL);
		}
		if((Last_LoB_Status == 1)&&(Current_LoB_Status == 0)){
			globalCounterL --;
			ROS_DEBUG("globalCounter : %d\n",globalCounterL);
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
	static unsigned char flagR;
	static unsigned char Last_RoB_Status;
	static unsigned char Current_RoB_Status;

	static int16_t globalCounterR = 0 ;
	Last_RoB_Status = digitalRead(RoBPin);
	
	volatile int temp_read = digitalRead(RoBPin);
	
	if(!temp_read){
		Current_RoB_Status = digitalRead(RoBPin);
		flagR = 1;
		return globalCounterR;
	}

	if(flagR == 1){
		flagR = 0;
		if((Last_RoB_Status == 0)&&(Current_RoB_Status == 1)){
			globalCounterR ++;
			ROS_DEBUG("globalCounter : %d\n",globalCounterR);
		}
		if((Last_RoB_Status == 1)&&(Current_RoB_Status == 0)){
			globalCounterR --;
			ROS_DEBUG("globalCounter : %d\n",globalCounterR);
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
