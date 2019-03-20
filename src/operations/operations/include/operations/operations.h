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

	std::stack<int> missions;
	achilles_slam::coord* start;
	ros::ServiceClient map_client; 
	ros::ServiceClient update_map_client;
	motor_abs::motor_driver* motor;

	void initialize(achilles_slam::course_map map);
	void traverse_to_empty(int curr_mission, achilles_slam::course_map map);
	void traverse_to_objective(achilles_slam::course_map map, achilles_slam::coord* dest);
	void grid_traverse(achilles_slam::course_map map);
	void objective_tasks();
	void mission_people();
	void mission_food();
	void mission_candle();
	achilles_slam::coord get_closest_unvisited(achilles_slam::course_map map);
	achilles_slam::course_map get_map();
	void update_tile(achilles_slam::coord coord, achilles_slam::course_map map);
	achilles_slam::coord* object_mapped(int object, achilles_slam::course_map map);
	std::vector<achilles_slam::coord> get_invalid(achilles_slam::course_map map, achilles_slam::coord* dest);
}

#endif