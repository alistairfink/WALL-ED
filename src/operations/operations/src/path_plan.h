#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#include "ros/ros.h"
#include <stack>

namespace path_plan {
	std::deque<achilles_slam::coord> path_plan_objective(achilles_slam::course_map map, achilles_slam::coord start, achilles_slam::coord dest, std::vector<achilles_slam::coord> invalid);
	std::deque<achilles_slam::coord> path_plan_grid(map map, pos start, std::vector invalid);

	class path_plan_helper
	{
	private:
		std::stack<achilles_slam::coords> neighbours;
		achilles_slam::coords tile;
	public:
		path_plan_helper(achilles_slam::coords current_tile, achilles_slam::coords previous_tile, std::uint16 width, std::vector<achilles_slam::coords> invalid);
		achilles_slam::coords get_next_neighbour();
	};
}

#endif