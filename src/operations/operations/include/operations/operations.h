#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#include <stack>
#include "ros/ros.h"
#include "operations/path_plan.h"
#include "achilles_slam/coord.h"
#include "motor_driver/motor_driver.h"
#include "achilles_slam/course_map.h"
#include "achilles_slam/get_course_map.h"

namespace operations {

	enum mission
	{
		candle = 6,
		food = 5,
		mansion = 3,
		cabin = 4,
	};

	int starting_tile;

	const int DIR_NORTH = 0;
	const int DIR_EAST = 1;
	const int DIR_SOUTH = 2;
	const int DIR_WEST = 3;

	std::stack<int> missions;
	achilles_slam::coord* start;
	ros::ServiceClient map_client; 
	ros::ServiceClient update_map_client;
	ros::ServiceClient controls_client;
	motor_abs::motor_driver* motor;
	int direction;
	std::stack<int> sand_index;
	std::vector<int> completed;

	void initialize(achilles_slam::course_map map, int start);
	void traverse_to_empty(int curr_mission, achilles_slam::course_map map);
	void traverse_to_objective(achilles_slam::course_map map, achilles_slam::coord* dest);
	void grid_traverse(achilles_slam::course_map map);
	void objective_tasks(uint8_t next);
	void mission_people(uint8_t next);
	void mission_food();
	void mission_candle();
	void turn_properly(achilles_slam::coord current, achilles_slam::coord next);
	achilles_slam::coord get_closest_unvisited(achilles_slam::course_map map);
	achilles_slam::course_map get_map();
	void update_tile(achilles_slam::coord coord, achilles_slam::course_map map);
	achilles_slam::coord* object_mapped(int object, achilles_slam::course_map map);
	std::vector<achilles_slam::coord> get_invalid(achilles_slam::course_map map, achilles_slam::coord* dest);
	int64_t use_controls(int64_t value);
	int get_next_target(achilles_slam::course_map map);
}

#endif