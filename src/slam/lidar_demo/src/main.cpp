#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "motor_driver/motor_driver.h"


motor_abs::motor_driver skye("/dev/ttyUSB1", 115200);


void stop_motor_maybe(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  static uint8_t count = 0;
  // ROS_INFO("Min angle: [%f]", msg->angle_min);
  // ROS_INFO("Max angle: [%f]", msg->angle_max);
  // ROS_INFO("Min range: [%f]", msg->range_min);
  // ROS_INFO("Max range: [%f]", msg->range_max);

  // ROS_INFO("Size: [%d]", int(msg->ranges.size()));
  // for (int i = 0; i < int(msg->ranges.size()) ; i++)
  // {
    // ROS_INFO("Index: [%d]", i);
    // ROS_INFO("Range: [%f]", msg->ranges[i]);
    // ROS_INFO("Intensity: [%f]", msg->intensities[i]);

  // }
  // ROS_INFO("================================\n");

  if (msg->ranges[180] < 0.5)
  {
    if (count < 6)
      count++;
  }
  else
    if (count > 0)
      count--;

  if (count > 3)
  {
    ROS_INFO("STOP!!");
    skye.set_speed(0,0);
  }

}

int main(int argc, char **argv)
{
  skye.set_speed(-100, 100);
  ros::init(argc, argv, "lidar_demo");
 
  ros::NodeHandle n;
  
  ros::Subscriber sub = n.subscribe("scan", 1000, stop_motor_maybe);

  ros::spin();

  return 0;
}