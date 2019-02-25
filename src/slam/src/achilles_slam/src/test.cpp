#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"


struct walls {
	uint32_t north_wall = 0xFFFFFFFF;
	uint32_t south_wall = 0xFFFFFFFF;
	uint32_t east_wall = 0xFFFFFFFF;
	uint32_t west_wall = 0xFFFFFFFF;
};

bool is_horizontal_wall(const uint8_t& data, const uint32_t width, const uint32_t row)
{
	uint32_t count = 0;

	// Loop throuh cells in given row
	for (uint32_t length = row * width ; count < width ; count++)
	{
		// Check length of continous adjacent cells and compare to.. width?
	}
}

walls identify_walls(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// Indices to search for walls
	uint32_t north_search = 0;
	uint32_t south_search = msg->info.width;

	// Used to break from while loop when both walls are found
	bool north_found = false
	bool south_found = false;

	// Loop through rows top down and bottom up simultaneously to find N and S walls
	while (!north_found && !south_found && north_search < south_search)
	{
		// CHECK IF EACH ROW CONTAINS HORIZONTAL WALL UNTIL ROWS OVERLAP OR BOTH ARE FOUND
	}
}

// Handle occupancy grids
void handle_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("Frame is [%d]", msg->header.seq);
	ROS_INFO("Resolution is [%f]", msg->info.resolution);
	ROS_INFO("Width is [%d]", msg->info.width);
	ROS_INFO("Height is [%d]\n", msg->info.height);
}



int main(int argc, char **argv)
{
  
	ros::init(argc, argv, "achilles_test");
	ros::NodeHandle n;

	// Subscribe to the /map for occupancy grids from Hector
	ros::Subscriber sub = n.subscribe("map", 1000, handle_map);
	ros::spin();

	return 0;
}