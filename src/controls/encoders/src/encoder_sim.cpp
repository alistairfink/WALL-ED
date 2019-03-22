#include <ros/ros.h>
#include "std_msgs/Int16.h"

#define PUB_FREQ 10 // Publish freq in Hz

int main(int argc, char **argv)
{
	ros::init(argc, argv, "encoder_sim");
	

	// Create node handle
    ros::NodeHandle n;

    // Encoder publishers
    ros::Publisher encoder_left_publisher = n.advertise<std_msgs::Int16>("lwheel", 100);
    ros::Publisher encoder_right_publisher = n.advertise<std_msgs::Int16>("rwheel", 100);

    ros::Rate loop_rate(PUB_FREQ); 

    int16_t encoder_value = -100;

    while (ros::ok())
    {

        std_msgs::Int16 lwheel_msg;
        std_msgs::Int16 rwheel_msg;

        lwheel_msg.data = encoder_value;
        rwheel_msg.data = encoder_value;
        encoder_value += 10;

        encoder_left_publisher.publish(lwheel_msg);
        encoder_right_publisher.publish(rwheel_msg);

        loop_rate.sleep();
    }

	return 0;
}