#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"

#define MAP_WIDTH 1.829			// Map width in meters
#define FOUND_ROW_FACTOR 2	// Divisor for expected # of cells to accept as wall


struct walls {
	// Set walls to max length of map cause will we ever actually use max length
	uint32_t north_wall = 0xFFFFFFFF;
	uint32_t south_wall = 0xFFFFFFFF;
	uint32_t east_wall = 0xFFFFFFFF;
	uint32_t west_wall = 0xFFFFFFFF;
};

uint32_t count_horizontal_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t row)
{
	uint64_t end = (row * msg->info.width) + msg->info.width; // last cell in that row
	uint32_t count = 0;

	// Loop throuh cells in given row and count occupied cells
	for (uint64_t cell = row * msg->info.width ; cell < end ; cell++)
	{
		if (msg->data[cell] == 100) // This is 100% probability.. same as what rviz displays
		{
			count++;
		}
	}
	return count;
}

uint32_t count_vertical_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t column)
{
	uint64_t end = column + (msg->info.width * (msg->info.width-1)); // last cell in that column
	uint32_t count = 0;

	// Loop throuh cells in given column and count occupied cells
	for (uint64_t cell = column ; cell < end ; cell += msg->info.width)
	{
		if (msg->data[cell] == 100) // This is 100% probability.. same as what rviz displays
		{
			count++;
		}
	}
	return count;
}

walls identify_walls(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	walls retVal;

	// Indices to search for walls
	uint32_t north_search = 0;
	uint32_t south_search = msg->info.width;

	// Used to break from while loop when both walls are found
	bool north_found = false;
	bool south_found = false;

	// Keeps track of count in row
	uint32_t north_count;
	uint32_t south_count;
	uint32_t last_north_count = 0;
	uint32_t last_south_count = 0;

	// Expected wall length in cells
	uint32_t actual_length = MAP_WIDTH / msg->info.resolution;

	// Loop through rows top down and bottom up simultaneously to find N and S walls
	// Loop until both walls found or search overlaps
	while (!north_found && !south_found && north_search < south_search)
	{
		// Search for North wall until found
		// Will compare # occupied cells in curr row with last, if # decreases and last row had > acceptance limit THEN found
		if (!north_found)
		{
			north_count = count_horizontal_cells(msg, north_search);
			if (north_count > last_north_count)
			{
				last_north_count = north_count;
			}
			else if (north_count < last_north_count && last_north_count >= actual_length / FOUND_ROW_FACTOR)
			{
				north_found = true;
				retVal.north_wall = north_search - 1;
			}

			north_search++;
		}

		// Search for South wall until found
		// Will compare # occupied cells in curr row with last, if # decreases and last row had > acceptance limit THEN found
		if (!south_found)
		{
			south_count = count_horizontal_cells(msg, north_search);
			if (south_count > last_south_count)
			{
				last_south_count = south_count;
			} else if (south_count < last_south_count && last_south_count >= actual_length / FOUND_ROW_FACTOR)
			{
				south_found = true;
				retVal.south_wall = south_search + 1;
			}
			
			south_search--;
		}
	}

	ROS_INFO("North wall is row [%d]", retVal.north_wall);
	ROS_INFO("South wall is row [%d]", retVal.south_wall);
	ROS_INFO("East wall is column [%d]", retVal.east_wall);
	ROS_INFO("West wall is column [%d]\n\n", retVal.west_wall);
	return retVal;
}

// Handle occupancy grids
void handle_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	ROS_INFO("Frame is [%d]", msg->header.seq);
	ROS_INFO("Resolution is [%f]", msg->info.resolution);
	ROS_INFO("Width is [%d]", msg->info.width);
	ROS_INFO("Height is [%d]\n", msg->info.height);

	identify_walls(msg);
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