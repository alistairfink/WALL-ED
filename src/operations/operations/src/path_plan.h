#ifndef OPERATIONS_H_
#define OPERATIONS_H_

#include "ros/ros.h"
#include <stack>

namespace path_plan {
	class path_plan_helper
	{
	private:
		std::stack<achilles_slam::coords> neighbours;
		achilles_slam::coords tile;
	public:
		path_plan_helper(achilles_slam::coords current_tile, std::vector<achilles_slam::coords> invalid);
		achilles_slam::coords get_next_neighbour();
	};
}

#endif