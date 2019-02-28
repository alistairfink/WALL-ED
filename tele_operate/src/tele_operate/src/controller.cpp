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

char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0) {
		//ROS_INFO("no_key_pressed");
	}
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");

	ros::NodeHandle n;
	ros::Publisher commands_pub = n.advertise<std_msgs::Int64>("tele_operate/command", 1);

	signal(SIGINT, quit);

	ros::Rate loop_rate(30);
	while (ros::ok())
	{
		int c = 0;
		c=getch();
    	std_msgs::Int64 msg;
    	switch(c)
    	{
			case 'w':
				ROS_INFO("FORWARD");
				msg.data = forward;
				break;
			case 's':
				ROS_INFO("BACK");
				msg.data = back;
				break;
			case 'a':
				ROS_INFO("LEFT");
				msg.data = turn_left;
				break;
			case 'd':
				ROS_INFO("RIGHT");
				msg.data = turn_right;
				break;
			default:
				ROS_INFO("NOTHING");
				msg.data = nothing;
    	}
    	
    	commands_pub.publish(msg);
		loop_rate.sleep();
	}

	return 0;
}