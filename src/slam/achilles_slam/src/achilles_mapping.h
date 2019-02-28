#ifndef ACHILLES_MAPPING_H
#define ACHILLES_MAPPING_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

struct walls;

uint32_t count_horizontal_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t row);

uint32_t count_vertical_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t column);

walls identify_walls(const nav_msgs::OccupancyGrid::ConstPtr& msg);

void handle_map(const nav_msgs::OccupancyGrid::ConstPtr& msg);

// terrain
enum terrain_type {	unknown, 
					flat, 
					gravel, 
					sand, 
					water };

// target
enum target_type {	unknown_undetermined, 
					none, 
					unknown_occupied, 
					mansion, 
					cabin, 
					supplies, 
					fire };

#endif