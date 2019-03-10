#include <stack>
#include "ros/ros.h"
#include "operations/path_plan.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"

using namespace path_plan;

std::deque<achilles_slam::coord> path_plan::path_plan_objective(
	achilles_slam::course_map map, 
	achilles_slam::coord start, 
	achilles_slam::coord dest, 
	std::vector<achilles_slam::coord> invalid)
{
	std::deque<achilles_slam::coord> path_plan;
	/*while(path_plan.back() != dest)
	{

	}*/

	return path_plan;
}

std::deque<achilles_slam::coord> path_plan::path_plan_grid(
	achilles_slam::course_map map, 
	achilles_slam::coord start, 
	std::vector<achilles_slam::coord> invalid)
{
	std::deque<achilles_slam::coord> path_plan;

	return path_plan;
}

path_plan_helper::path_plan_helper(
	achilles_slam::coord current_tile,
	achilles_slam::coord previous_tile,
	uint16_t width, 
	std::vector<achilles_slam::coord> invalid)
{
	tile = current_tile;
	if(current_tile.y != 0 &&
		current_tile.y - 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coord left;
		left.x = current_tile.x;
		left.y = current_tile.y - 1;
		neighbours.push(left);
	}

	if (current_tile.x != 0 &&
		current_tile.x - 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coord top;
		top.x = current_tile.x - 1;
		top.y = current_tile.y;
		neighbours.push(top);
	}

	if (current_tile.y != width -1 &&
		current_tile.y + 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coord right;
		right.x = current_tile.x;
		right.y = current_tile.y + 1;
		neighbours.push(right);
	}

	if (current_tile.x != width -1 &&
		current_tile.x + 1 != previous_tile.x /*&& CHeck if invalid has this node*/)
	{
		achilles_slam::coord bottom;
		bottom.x = current_tile.x + 1;
		bottom.y = current_tile.y;
		neighbours.push(bottom);
	}
}
	
achilles_slam::coord path_plan_helper::get_next_neighbour()
{
	achilles_slam::coord next = neighbours.pop();
	return next;
}