#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "achilles_slam/course_map.h"
#include "achilles_slam/get_course_map.h"
#include "achilles_slam/update_course_map.h"
#include "achilles_mapping.h"

#define MAP_WIDTH_M 1.829		// Map width in meters
#define MAP_WIDTH_TILES 6		// Map width in tiles
#define FOUND_ROW_FACTOR 2	// Divisor for expected # of cells to accept as wall


/**
* Walls struct
* Contains row/column of walls in occupancy grid from hector.
* Possible wall row and column numbers start at 0 (from left and top of occupancy grid array respectively) and go to (width-1).
*/
struct achilles_mapping_service::walls {
	// Set walls to max length of map cause will we ever actually use max length. These will mean wall not found.
	uint32_t north_wall = 0xFFFFFFFF;
	uint32_t south_wall = 0xFFFFFFFF;
	uint32_t east_wall = 0xFFFFFFFF;
	uint32_t west_wall = 0xFFFFFFFF;
};


/**
* count_horizontal_cells
* Takes a row number (0 to width-1 left to right) and returns the count of occupied cells in that row of occupancy grid.
*
* @param msg const pointer to nav_msgs::OccupancyGrid containing occupancy grid as 1D array
* @param row integer of row to count occupied cells. (Value should be 0 to width-1)
* @return The number of occupied cells in row
*/
uint32_t achilles_mapping_service::count_horizontal_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t row)
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


/**
* count_vertical_cells
* Takes a column number (0 to width-1 top to bottom) and returns the count of occupied cells in that column of occupancy grid.
*
* @param msg const pointer to nav_msgs::OccupancyGrid containing occupancy grid as 1D array
* @param row integer of column to count occupied cells. (Value should be 0 to width-1)
* @return The number of occupied cells in column
*/
uint32_t achilles_mapping_service::count_vertical_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t column)
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


/**
* identify_walls
* Processes occupancy grid from hector_mapping and determines rows and columns of walls of obstacle course
*
* @param msg const pointer to nav_msgs::OccupancyGrid containing occupancy grid as 1D array
* @return An instance of wall struct containing row and column numbers of walls. 
*/
achilles_mapping_service::walls achilles_mapping_service::identify_walls(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	walls retVal;

	// Indices to search for walls
	// Row number
	uint32_t north_search = 0;
	uint32_t south_search = msg->info.width - 1;

	// Col number
	uint32_t east_search = msg->info.width -1;
	uint32_t west_search = 0;

	// Used to break from while loop when both walls are found
	bool north_found = false;
	bool south_found = false;
	bool east_found = false;
	bool west_found = false;

	// Keeps track of count in row
	uint32_t north_count;
	uint32_t south_count;
	uint32_t east_count;
	uint32_t west_count;
	uint32_t last_north_count = 0;
	uint32_t last_south_count = 0;
	uint32_t last_east_count = 0;
	uint32_t last_west_count = 0;

	// Expected wall length in cells
	uint32_t actual_length = MAP_WIDTH_M / msg->info.resolution;

	// Loop through rows top down and bottom up simultaneously to find N and S walls
	// =====================================================================================
	// Loop until both walls found or search overlaps
	while ((!north_found || !south_found) && (north_search < south_search))
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
			south_count = count_horizontal_cells(msg, south_search);
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
	// =====================================================================================

	// Loop through rows right->left and left->right simultaneously to find E and W walls
	// =====================================================================================
	// Loop until both walls found or search overlaps
	while ((!east_found || !west_found) && (west_search < east_search))
	{
		// Search for West wall until found
		// Will compare # occupied cells in curr col with last, if # decreases and last col had > acceptance limit THEN found
		if (!west_found)
		{
			west_count = count_vertical_cells(msg, west_search);
			if (west_count > last_west_count)
			{
				last_west_count = west_count;
			}
			else if (west_count < last_west_count && last_west_count >= actual_length / FOUND_ROW_FACTOR)
			{
				west_found = true;
				retVal.west_wall = west_search - 1;
			}

			west_search++;
		}

		// Search for East wall until found
		// Will compare # occupied cells in curr col with last, if # decreases and last col had > acceptance limit THEN found
		if (!east_found)
		{
			east_count = count_vertical_cells(msg, east_search);
			if (east_count > last_east_count)
			{
				last_east_count = east_count;
			}
			else if (east_count < last_east_count && last_east_count >= actual_length / FOUND_ROW_FACTOR)
			{
				east_found = true;
				retVal.east_wall = east_search + 1;
			}

			east_search--;
		}
	}
	// =====================================================================================


	// If one wall is marked found and opposite isnt then estimate using obstacle course length
	// ===========================================================================================
	if (north_found && !south_found)
	{
		retVal.south_wall = retVal.north_wall + actual_length;
	} else if (!north_found && south_found) 
	{
		retVal.north_wall = retVal.south_wall - actual_length;
	}

	if (east_found && !west_found)
	{
		retVal.west_wall = retVal.east_wall - actual_length;
	} else if (!east_found && west_found) 
	{
		retVal.east_wall = retVal.west_wall + actual_length;
	}
	// ===========================================================================================


	ROS_INFO("North wall is row [%d]", retVal.north_wall);
	ROS_INFO("South wall is row [%d]", retVal.south_wall);
	ROS_INFO("East wall is column [%d]", retVal.east_wall);
	ROS_INFO("West wall is column [%d]\n\n", retVal.west_wall);
	return retVal;
}



achilles_mapping_service::achilles_mapping_service()
{
	// Course map for this instance
	this->course_map = new achilles_slam::course_map;

	// Write width to return message
	this->course_map->width = MAP_WIDTH_TILES;

	// Tile holder to be pushed into map vector
	achilles_slam::tile temp_tile;
	temp_tile.terrain = achilles_slam::tile::TERRAIN_UKNOWN;
	temp_tile.target = achilles_slam::tile::TARGET_UKNOWN_UNDERTERMINED;

	for (uint16_t i = 0 ; i < MAP_WIDTH_TILES*MAP_WIDTH_TILES ; i++)
	{
		temp_tile.terrain = 0;
		this->course_map->map.push_back(temp_tile);
	}
}

achilles_mapping_service::~achilles_mapping_service()
{
	delete this->course_map;
}

bool achilles_mapping_service::get_course_map_srv(achilles_slam::get_course_map::Request& req, achilles_slam::get_course_map::Response& resp)
{
}

bool achilles_mapping_service::update_course_map_srv(achilles_slam::update_course_map::Request& req, achilles_slam::update_course_map::Response& resp)
{
	
}








// target_type achilles_mapping_service::process_tile_target(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint16_t tile_num) 
// {

// }

// achilles_slam::course_map achilles_mapping_service::construct_course_map(const nav_msgs::OccupancyGrid::ConstPtr& msg, const walls course_walls)
// {
// 	// Course Map to be returned
// 	achilles_slam::course_map retVal;

// 	// Write width to return message
// 	retVal.width = MAP_WIDTH_TILES;

// 	// Tile holder to be pushed into map vector
// 	achilles_slam::tile temp_tile;

// 	for (uint16_t i = 0 ; i < MAP_WIDTH_TILES*MAP_WIDTH_TILES ; i++)
// 	{
// 		temp_tile;
// 		retVal.map.push_back(temp_tile);
// 	}
// }


// /**
// * handle_map
// * Callback for arriving msgs on the map topic. Hands occupancy grid to relevant functions for processing to course map.
// *
// * @param msg const pointer to nav_msgs::OccupancyGrid containing occupancy grid as 1D array
// */
// void handle_map(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// {	
// 	walls course_walls;
// 	// ROS_INFO("Frame is [%d]", msg->header.seq);
// 	// ROS_INFO("Resolution is [%f]", msg->info.resolution);
// 	// ROS_INFO("Width is [%d]", msg->info.width);
// 	// ROS_INFO("Height is [%d]\n", msg->info.height);

// 	course_walls = identify_walls(msg);

// 	construct_course_map(msg, course_walls);
// }

