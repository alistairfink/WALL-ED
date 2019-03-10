#ifndef PATH_PLAN_H_
#define PATH_PLAN_H_

#include <stack>
#include "ros/ros.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"

namespace path_plan 
{
	std::deque<achilles_slam::coord> path_plan_objective(achilles_slam::course_map map, achilles_slam::coord start, achilles_slam::coord dest, std::vector<achilles_slam::coord> invalid);

	std::deque<achilles_slam::coord> path_plan_grid(achilles_slam::course_map map, achilles_slam::coord start, std::vector<achilles_slam::coord> invalid);

	class path_plan_helper
	{
	public:
		std::stack<achilles_slam::coord> neighbours;
		achilles_slam::coord tile;
		path_plan_helper(achilles_slam::coord current_tile, achilles_slam::coord previous_tile, uint16_t width, std::vector<achilles_slam::coord> invalid);
		achilles_slam::coord get_next_neighbour();
	};
}

#endif