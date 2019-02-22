#include "ros/ros.h"
#include "std_msgs/Int64.h"

#include <sstream>
#include <termios.h>
#include <signal.h>

#include "commands.h"

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

struct termios cooked, raw;
int kfd = 0;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");

	ros::NodeHandle n;
	ros::Publisher commands_pub = n.advertise<std_msgs::Int64>("tele_operate/command", 1);

	signal(SIGINT, quit);

	char c;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);                       
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);
	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the WALL-ED.");

	bool dirty = false;
	while (ros::ok())
	{  
	    if(read(kfd, &c, 1) < 0)
	    {
	      perror("read():");
	      exit(-1);
	    }

    	ROS_DEBUG("value: 0x%02X\n", c);

    	std_msgs::Int64 msg;
    	switch(c)
    	{
			case KEYCODE_U:
				ROS_DEBUG("FORWARD");
				msg.data = forward;
				dirty = true;
				break;
			case KEYCODE_D:
				ROS_DEBUG("BACK");
				msg.data = back;
				dirty = true;
				break;
			case KEYCODE_L:
				ROS_DEBUG("LEFT");
				msg.data = turn_left;
				dirty = true;
				break;
			case KEYCODE_R:
				ROS_DEBUG("RIGHT");
				msg.data = turn_right;
				dirty = true;
				break;
    	}


    	if (dirty) 
    	{
	    	commands_pub.publish(msg);
    		dirty = false;
    	}

		/*std_msgs::String msg;
		std::stringstream ss;
		ss << 1;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());
		chatter_pub.publish(msg);
		loop_rate.sleep()*/;
	}

	return 0;
}