#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "achilles_slam/course_map.h"
#include "achilles_slam/get_course_map.h"
#include "achilles_slam/update_course_map.h"
#include "achilles_mapping.h"
#include <math.h>

#define UNFOUND 0xFFFFFFFF


/**
* Walls struct
* Contains row/column of walls in occupancy grid from hector.
* Possible wall row and column numbers start at 0 (from left and top of occupancy grid array respectively) and go to (width-1).
*/
struct achilles_mapping_service::walls {
	// Set walls to max length of map cause will we ever actually use max length. These will mean wall not found.
	uint32_t north_wall = UNFOUND;
	uint32_t south_wall = UNFOUND;
	uint32_t east_wall = UNFOUND;
	uint32_t west_wall = UNFOUND;
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

	// ROS_DEBUG("Row: %d\tCount: %d", row, count);
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
	// ROS_DEBUG("Column: %d\tCount: %d", column, count);
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
	uint32_t actual_length = this->map_width_m / msg->info.resolution;

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
			else if (north_count < last_north_count && last_north_count >= actual_length / this->found_wall_factor)
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
			} else if (south_count < last_south_count && last_south_count >= actual_length / this->found_wall_factor)
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
			else if (west_count < last_west_count && last_west_count >= actual_length / this->found_wall_factor)
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
			else if (east_count < last_east_count && last_east_count >= actual_length / this->found_wall_factor)
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


	ROS_DEBUG("North wall is row [%d]", retVal.north_wall);
	ROS_DEBUG("South wall is row [%d]", retVal.south_wall);
	ROS_DEBUG("East wall is column [%d]", retVal.east_wall);
	ROS_DEBUG("West wall is column [%d]\n\n", retVal.west_wall);
	return retVal;
}


/**
* achilles_mapping_service
* Constructor for the achilles mapping serivce. Constructs empty course_map, reads node params and launches services
*/
achilles_mapping_service::achilles_mapping_service()
{
	// Course map for this instance
	this->course_map = new achilles_slam::course_map;

	// Create node handle
	ros::NodeHandle n;

	// Handle parameters
	n.param<bool>("/achilles_mapping/ignore_edge_cells", this->ignore_edge_cells, false);	// Ignore occupied cells adjacent to walls
	n.param<float>("/achilles_mapping/map_width_m", this->map_width_m, 1.829);				// Map width in meters
	n.param<int>("/achilles_mapping/map_width_tiles", this->map_width_tiles, 6);			// Map width in tiles
	n.param<int>("/achilles_mapping/found_wall_factor", this->found_wall_factor, 2);		// Divisor for expected # of cells to accept as wall
	n.param<float>("/achilles_mapping/min_target_area", this->min_target_area, 0.001);		// Threshold area in m^2 to mark a tile as "occupied" e.g. 0.001 = 10cm^2
	n.param<float>("/achilles_mapping/min_unknown_area", this->min_unknown_area, 0.002);	// Threshold area in m^2 to mark a tile as "unknown" e.g. 0.001 = 10cm^2
	n.param<int>("/start_position", this->start_pos, 1);									// Start position in the course

	// Write width to return message
	this->course_map->width = this->map_width_tiles;

	this->course_map->robot_pos.x = this->map_width_tiles;
	this->course_map->robot_pos.y = this->map_width_tiles;

	// Tile holder to be pushed into map vector
	achilles_slam::tile temp_tile;
	temp_tile.terrain = achilles_slam::tile::TERRAIN_FLAT;
	temp_tile.target = achilles_slam::tile::TARGET_UKNOWN_UNDERTERMINED;
	temp_tile.visited = false;

	// Set initialize map to null tiles
	for (uint16_t i = 0 ; i < this->map_width_tiles * this->map_width_tiles ; i++)
	{
		this->course_map->map.push_back(temp_tile);
	}

	// Set tile terrains
	if (this->start_pos == 1)
	{
		// ROCKS
		this->course_map->map[2].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[17].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[25].terrain = achilles_slam::tile::TERRAIN_GRAVEL;

		// SAND
		this->course_map->map[7].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[15].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[28].terrain = achilles_slam::tile::TERRAIN_SAND;

		// DROPS
		this->course_map->map[10].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[18].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[33].terrain = achilles_slam::tile::TERRAIN_WATER;

		uint8_t temp_bad_zones[6] = {2, 17, 25, 10, 18, 33};
		this->course_map->bad_zones.insert(this->course_map->bad_zones.end(), temp_bad_zones, temp_bad_zones+6);
	} 
	else if (this->start_pos == 2)
	{
		// ROCKS
		this->course_map->map[2].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[18].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[28].terrain = achilles_slam::tile::TERRAIN_GRAVEL;

		// SAND
		this->course_map->map[10].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[14].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[25].terrain = achilles_slam::tile::TERRAIN_SAND;

		// DROPS
		this->course_map->map[7].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[17].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[33].terrain = achilles_slam::tile::TERRAIN_WATER;

		uint8_t temp_bad_zones[6] = {2, 18, 28, 7, 17, 33};
		this->course_map->bad_zones.insert(this->course_map->bad_zones.end(), temp_bad_zones, temp_bad_zones+6);

	}
	else if (this->start_pos == 3)
	{
		// ROCKS
		this->course_map->map[10].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[18].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[33].terrain = achilles_slam::tile::TERRAIN_GRAVEL;

		// SAND
		this->course_map->map[7].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[20].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[28].terrain = achilles_slam::tile::TERRAIN_SAND;

		// DROPS
		this->course_map->map[17].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[2].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[25].terrain = achilles_slam::tile::TERRAIN_WATER;

		uint8_t temp_bad_zones[6] = {10, 18, 33, 17, 2, 25};
		this->course_map->bad_zones.insert(this->course_map->bad_zones.end(), temp_bad_zones, temp_bad_zones+6);

	}
	else if (this->start_pos == 4)
	{
		// ROCKS
		this->course_map->map[17].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[7].terrain = achilles_slam::tile::TERRAIN_GRAVEL;
		this->course_map->map[33].terrain = achilles_slam::tile::TERRAIN_GRAVEL;

		// SAND
		this->course_map->map[10].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[21].terrain = achilles_slam::tile::TERRAIN_SAND;
		this->course_map->map[25].terrain = achilles_slam::tile::TERRAIN_SAND;

		// DROPS
		this->course_map->map[2].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[18].terrain = achilles_slam::tile::TERRAIN_WATER;
		this->course_map->map[28].terrain = achilles_slam::tile::TERRAIN_WATER;

		uint8_t temp_bad_zones[6] = {17, 7, 33, 2, 18, 28};
		this->course_map->bad_zones.insert(this->course_map->bad_zones.end(), temp_bad_zones, temp_bad_zones+6);

	}

	this->robot_cell = UNFOUND;

	// Launch map services
	this->get_serv = n.advertiseService("get_course_map", &achilles_mapping_service::get_course_map_srv, this);
	this->update_serv = n.advertiseService("update_course_map", &achilles_mapping_service::update_course_map_srv, this);
}


/**
* ~achilles_mapping_service
* Destructor for achilles_mapping_service class.
*
* Just deallocates the map
*/
achilles_mapping_service::~achilles_mapping_service()
{
	// delete the map
	delete this->course_map;
}

/**
* process_tile
* Processes tile from occupancy grid and returns tile object. 
*
* @param msg const pointer to nav_msgs::OccupancyGrid containing occupancy grid as 1D array
* @param course_walls const pointer to achilles_mapping_service::walls rows and columns of walls
* @param tile_num integer of the index of the tile to process.
* @return An instance of achilles_slam::tile containing the processed tile. 
*/
achilles_slam::tile achilles_mapping_service::process_tile(const nav_msgs::OccupancyGrid::ConstPtr &msg, const achilles_mapping_service::walls *course_walls, const uint16_t tile_num) 
{
	ROS_DEBUG("TILE: [%d]", tile_num);
	achilles_slam::tile processed_tile;

	// Counts of cell states in tile
	uint32_t occupancy_count = 0;
	uint32_t unknown_count = 0;
	uint32_t empty_count = 0;

	// Determine length and width of one tile
	// determined separately in case detected walls do not form perfect square

	// Tile length stuff
	// ==================================================================================================
	float tile_length = (course_walls->south_wall - course_walls->north_wall - 1) / (float)this->map_width_tiles;
	uint32_t effective_tile_length;
	// check if cells per tile length fit perfectly or nah
	if ((uint32_t)(tile_length*10)%10 == 0)
	{
		effective_tile_length = tile_length;
	}
	else
	{
		// If its a tile on the edge of the map dont include wall in tile length
		if (tile_num/this->map_width_tiles == 0 || floor(tile_num/this->map_width_tiles) == this->map_width_tiles-1)
			effective_tile_length = ceil(tile_length);
		else
			effective_tile_length = ceil(tile_length) + 1;
	}
	// ==================================================================================================


	// Tile width stuff
	// ==================================================================================================
	float tile_width = (course_walls->east_wall - course_walls->west_wall - 1) / (float)this->map_width_tiles;
	uint32_t effective_tile_width;
	// check if cells per tile row fit perfectly or nah
	if ((uint32_t)(tile_width*10)%10 == 0)
	{
		effective_tile_width = tile_width;
	}
	else
	{
		// If its a tile on the edge of the map dont include wall in tile width
		if (tile_num%this->map_width_tiles == 0 || tile_num%this->map_width_tiles == this->map_width_tiles-1)
			effective_tile_width = ceil(tile_width);
		else
			effective_tile_width = ceil(tile_width) + 1;
	}
	// ==================================================================================================

	// First cell in the tile
	// +++++++++++++++++++++++++++++
	// Calc cell pos of first cell in row
	uint32_t start_cell =   (  (course_walls->north_wall + 1)  +  ((tile_num/this->map_width_tiles) ? ((ceil(tile_length) * (tile_num/this->map_width_tiles))-1) : 0 ))   *   msg->info.width ;
	// Calc cell position of cell's col in row
	start_cell += (course_walls->west_wall + 1) + ( tile_num%this->map_width_tiles ? (ceil(tile_width) * (tile_num%this->map_width_tiles) - 1) : 0 );
	// +++++++++++++++++++++++++++++

	// Index of cell to check
	uint32_t row_start_cell = 0;
	uint32_t curr_cell = 0;

	uint32_t start_row = 0;
	uint32_t start_col = 0;

	// Set robot pos
	if (this->robot_cell%msg->info.width >= start_cell%msg->info.width && this->robot_cell%msg->info.width < start_cell%msg->info.width + effective_tile_width)
	{
		if (this->robot_cell/msg->info.height >= start_cell/msg->info.height && this->robot_cell/msg->info.height < start_cell/msg->info.height + effective_tile_length)
		{
			this->course_map->robot_pos.x = tile_num%this->map_width_tiles;
			this->course_map->robot_pos.y = tile_num/this->map_width_tiles;
		}
	}

	// Ignore cells adjacent to walls if param set
	if (this->ignore_edge_cells)
	{
		if (tile_num/this->map_width_tiles == 0)
		{
			// Top row
			start_row = 1;
		}
		else if (tile_num/this->map_width_tiles + 1 == tile_num/this->map_width_tiles)
		{
			// Bottom row
			effective_tile_length--;
		}
		if (tile_num%this->map_width_tiles == 0)
		{
			// First column
			start_col = 1;
		}
		else if (tile_num%this->map_width_tiles + 1 == tile_num/this->map_width_tiles)
		{
			// Last column
			effective_tile_width--;
		}
	}
		
	// Cycle through rows of tile
	// ===========================================
	for (uint32_t row_count = start_row ; row_count < effective_tile_length ; row_count++)
	{
		row_start_cell = start_cell + (msg->info.width * row_count);
		
		// Cycle through columns of row 
		for (uint32_t column_count = start_col ; column_count < effective_tile_width ; column_count++)
		{
			curr_cell = row_start_cell + column_count;

			if (msg->data[curr_cell] == 100)
				ROS_DEBUG("cell: %d  !", curr_cell);
			else
				ROS_DEBUG("cell: %d", curr_cell);

			// Count occupied/empty/unknown cells
			if (msg->data[curr_cell] == 100)
			{
				occupancy_count++;
			} 
			else if (msg->data[curr_cell] == -1)
			{
				unknown_count++;
			}
			else
			{
				empty_count++;
			}
		}
		ROS_DEBUG("- New row -");
	}
	// ===========================================

	processed_tile.terrain = achilles_slam::tile::TERRAIN_UKNOWN;
	processed_tile.occupancy_count = occupancy_count;

	// If occupancy_count > occupancy acceptance theshold then set occupied
	// else likewise for unknown_count
	if ((float)(occupancy_count *  msg->info.resolution * msg->info.resolution) > float(this->min_target_area))
	{
		processed_tile.target = achilles_slam::tile::TARGET_OCCUPIED;
	} 
	else if (unknown_count  *  msg->info.resolution * msg->info.resolution > this->min_unknown_area)
	{
		processed_tile.target = achilles_slam::tile::TARGET_UKNOWN_UNDERTERMINED;
	} 
	else
	{
		processed_tile.target = achilles_slam::tile::TARGET_NONE;
	}

	
	// Debug row output
	ROS_DEBUG("Start column: [%d]", start_cell%msg->info.width);
	ROS_DEBUG("Tile width: [%d]", effective_tile_width);
	ROS_DEBUG("End column: [%d]", start_cell%msg->info.width + effective_tile_width - 1 );
	ROS_DEBUG("---");

	// Debug row output
	ROS_DEBUG("Start row: [%d]", start_cell/msg->info.width);
	ROS_DEBUG("Tile length: [%d]", effective_tile_length);
	ROS_DEBUG("End row: [%d]", start_cell/msg->info.width + effective_tile_length - 1 );
	ROS_DEBUG("---");

	// Occupancy info
	ROS_DEBUG("occupancy_count: [%d]", occupancy_count);
	ROS_DEBUG("occupancy area: [%f]", occupancy_count *  msg->info.resolution * msg->info.resolution);
	ROS_DEBUG("Min req occ: [%f]", this->min_target_area);
	ROS_DEBUG("===================");

	return processed_tile;
}


/**
* get_robot_cell
* Find the cell the robot is in
*
* @return cell of robot
*/
uint32_t achilles_mapping_service::get_robot_cell(const nav_msgs::OccupancyGrid::ConstPtr &occ_grid)
{
	geometry_msgs::PoseStamped::ConstPtr pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("slam_out_pose", ros::Duration(2));
	if (pose == NULL)
	{
		ROS_INFO("No pose data");
	}

	uint32_t curr_cell = floor((pose->pose.position.y - occ_grid->info.origin.position.y ) / occ_grid->info.resolution) * occ_grid->info.width;
	curr_cell += floor((pose->pose.position.x - occ_grid->info.origin.position.x ) / occ_grid->info.resolution);

	return curr_cell;
}



/**
* get_course_map_srv
* Service to get the course map 
*
* @param req Its pretty much nothing. the request is empty
* @param resp A copy of the object's course_map
* @return true.. sometimes. if not false.
*/
bool achilles_mapping_service::get_course_map_srv(achilles_slam::get_course_map::Request& req, achilles_slam::get_course_map::Response& resp)
{
	// Get occupancy grid from hector and make sure its not null
	nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(2));
	if (msg == NULL)
	{
        ROS_INFO("No point occupancy grid messages received");
        return false;
	}
    else
    {
    	// Tile used for processing occ grid
    	achilles_slam::tile temp_tile;

    	// Identify walls of obtained occupancy grid
        achilles_mapping_service::walls course_walls = this->identify_walls(msg);

        this->robot_cell = this->get_robot_cell(msg);

        // If walls not found return empty map
        if (course_walls.north_wall == UNFOUND || course_walls.east_wall == UNFOUND)
        {
        	resp.silicon_valley = *(this->course_map);
			return true;
        }

        // Loop through tiles and process
		for (uint16_t tile_num = 0 ; tile_num < this->map_width_tiles*this->map_width_tiles ; tile_num++)
		{
			temp_tile = this->process_tile(msg, &course_walls, tile_num);

			// Update target only if we havent determined target yet.
			this->course_map->map[tile_num].occupancy_count = temp_tile.occupancy_count;

			// Only update if tile is unvisited
			if (!this->course_map->map[tile_num].visited)
			{
				// If we a tile was marked as occupied and changing to empty
				if (this->course_map->map[tile_num].target > achilles_slam::tile::TARGET_NONE && temp_tile.target <= achilles_slam::tile::TARGET_NONE)
				{
					for (std::vector<uint8_t>::iterator it = this->course_map->target_list.begin() ; it < this->course_map->target_list.end() ; it++)
					{
						if (*it == tile_num)
						{
							it = this->course_map->target_list.erase(it);
							break;
						}
					}
				}
				// If we a tile was unoccupied/unknown and changing to empty
				if (this->course_map->map[tile_num].target <= achilles_slam::tile::TARGET_NONE && temp_tile.target > achilles_slam::tile::TARGET_NONE)
				{
					this->course_map->target_list.push_back(tile_num);
				}
				this->course_map->map[tile_num].target = temp_tile.target;

				// Note we dont update the terrain here cause lidar doesnt tell us that
			}
				
		}
	    
	    // response from service
	    resp.silicon_valley = *(this->course_map);
	}

	return true; // cause why not lol
}

/**
* update_course_map_srv
* Service to update the course map 
*
* @param req a copy of the tile to update and it's index
* @param resp nothing. no response required
* @return Always true i guess. 
*/
bool achilles_mapping_service::update_course_map_srv(achilles_slam::update_course_map::Request& req, achilles_slam::update_course_map::Response& resp)
{
	this->course_map->map[req.tile_coord].terrain = req.update_tile.terrain;
	this->course_map->map[req.tile_coord].target = req.update_tile.target;
	this->course_map->map[req.tile_coord].visited = req.update_tile.visited;
	return true;
}



