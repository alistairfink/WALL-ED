#include "ros/ros.h"
#include "wiringPi.h"
#include "encoder_handler.h"
#include "std_msgs/Int16.h"

// Pi the pins connect
#define  LEFT_A_PIN		0
#define  LEFT_B_PIN		1
#define  RIGHT_A_PIN	2
#define  RIGHT_B_PIN	26
#define  RoSPin			24

#define PUB_FREQ 200 	// Publish freq in Hz


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
	pinMode(RIGHT_A_PIN, INPUT);
	pinMode(RIGHT_B_PIN, INPUT);
	pinMode(LEFT_A_PIN, INPUT);
	pinMode(LEFT_B_PIN, INPUT);

	// Create node handle
	ros::NodeHandle n;

	// Encoder publishers
	ros::Publisher encoder_left_publisher = n.advertise<std_msgs::Int16>("lwheel", 100);
	ros::Publisher encoder_right_publisher = n.advertise<std_msgs::Int16>("rwheel", 100);

	// Get the freq param
	int pub_frequency;
	n.param<int>("/encoder_handler/pub_frequency", pub_frequency, 200);

	ros::Rate loop_rate(pub_frequency); 

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


// FOR DEBUG
// =====================================
// unsigned char get_a()
// {
// 	static int count = 0;

// 	count = count % 4;

// 	switch (count++){
// 		case 0: return 0;
// 		break;
// 		case 1: return 1;
// 		break;
// 		case 2: return 1;
// 		break;
// 		case 3: return 0;
// 		break;
// 	}
// }
// unsigned char get_b()
// {
// 	static int count = 0;

// 	count = count % 4;

// 	switch (count++){
// 		case 0: return 0;
// 		break;
// 		case 1: return 0;
// 		break;
// 		case 2: return 1;
// 		break;
// 		case 3: return 1;
// 		break;
// 	}
// }
// =====================================



/**
* motor_l
* Gets encoder reading for left motor
*
* @return signed 16bit int for encoder reading
*/
int16_t encoder_handler::motor_l()
{	
	static int16_t encoder_count = 0 ;

	// FOR DEBUG
	// volatile unsigned char curr_a_state = get_a();
	// volatile unsigned char curr_b_state = get_b();

	volatile unsigned char curr_b_state = digitalRead(RIGHT_B_PIN);

	static unsigned char last_b_state = curr_b_state;

	if (curr_b_state == last_b_state)
	{
		return encoder_count;
	}
	else
	{
		volatile unsigned char curr_a_state = digitalRead(RIGHT_A_PIN);
		if (curr_b_state == curr_a_state)
		{
			encoder_count++;
		}
		else
		{
			encoder_count--;
		}

		last_b_state = curr_b_state;
		return encoder_count;
	}
}

/**
* motor_r
* Gets encoder reading for right motor
*
* @return signed 16bit int for encoder reading
*/
int16_t encoder_handler::motor_r()
{
	static int16_t encoder_count = 0 ;

	// FOR DEBUG
	// volatile unsigned char curr_a_state = get_a();
	// volatile unsigned char curr_b_state = get_b();

	volatile unsigned char curr_b_state = digitalRead(LEFT_B_PIN);

	static unsigned char last_b_state = curr_b_state;

	if (curr_b_state == last_b_state)
	{
		return encoder_count;
	}
	else
	{
		volatile unsigned char curr_a_state = digitalRead(LEFT_A_PIN);
		if (curr_b_state == curr_a_state)
		{
			encoder_count++;
		}
		else
		{
			encoder_count--;
		}

		last_b_state = curr_b_state;
		return encoder_count;
	}
}


/**
* rotary_clear
* Clears current encoder reading?
*/
void encoder_handler::rotary_clear()
{
	if(digitalRead(RoSPin) == 0)
	{
		int globalCounter = 0;
		ROS_DEBUG("globalCounter : %d\n",globalCounter);
		delay(1000);
	}
}
