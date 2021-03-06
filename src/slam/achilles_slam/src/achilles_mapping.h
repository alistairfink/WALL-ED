#ifndef ACHILLES_MAPPING_H
#define ACHILLES_MAPPING_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "achilles_slam/course_map.h"
#include "achilles_slam/get_course_map.h"
#include "achilles_slam/update_course_map.h"


class achilles_mapping_service {
public:

	achilles_mapping_service();
	~achilles_mapping_service();

	bool get_course_map_srv(achilles_slam::get_course_map::Request& req, achilles_slam::get_course_map::Response& resp);
	bool update_course_map_srv(achilles_slam::update_course_map::Request& req, achilles_slam::update_course_map::Response& resp);

private:
	achilles_slam::course_map *course_map;

	struct walls;

	uint32_t robot_cell;

	uint32_t get_robot_cell(const nav_msgs::OccupancyGrid::ConstPtr &occ_grid);

	uint32_t count_horizontal_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t row);

	uint32_t count_vertical_cells(const nav_msgs::OccupancyGrid::ConstPtr& msg, const uint32_t column);

	walls identify_walls(const nav_msgs::OccupancyGrid::ConstPtr& msg);

	achilles_slam::tile process_tile(const nav_msgs::OccupancyGrid::ConstPtr &msg, const achilles_mapping_service::walls *course_walls, const uint16_t tile_num);

	ros::ServiceServer get_serv;
	ros::ServiceServer update_serv;

	// Node parameters
	float map_width_m;
	int map_width_tiles;
	int found_wall_factor;
	float min_target_area;
	float min_unknown_area;
	bool ignore_edge_cells;
	int start_pos;
};




#endif