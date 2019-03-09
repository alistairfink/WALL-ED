#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#include <stack>
#include "ros/ros.h"
#include "operations/operations.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"

namespace operations {

	enum mission
	{
		candle = 6,
		food = 5,
		mansion = 3,
		cabin = 4,
	};

	std::stack<int> missions;

	void initialize();
	void traverse_to_empty(int curr_mission, achilles_slam::course_map map, achilles_slam::coord curr_pos);
	void traverse_to_objective(int curr_mission);
	void grid_traverse();
	void objective_tasks();
	void mission_people();
	void mission_food();
	void mission_candle();
	achilles_slam::coord object_mapped(int object, achilles_slam::course_map map)
}

#endif