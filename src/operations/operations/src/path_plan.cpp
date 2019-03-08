#include "ros/ros.h"
#include "./path_plan.h"
#include <stack>

using namespace path_plan;

std::queue<achilles_slam::coord> path_plan_objective(
	achilles_slam::course_map map, 
	achilles_slam::coord start, 
	achilles_slam::coord dest, 
	std::vector<achilles_slam::coord> invalid)
{

}

std::queue<achilles_slam::coord> path_plan_grid(
	map map, 
	pos start, 
	std::vector invalid)
{

}

path_plan_helper::path_plan_helper(
	achilles_slam::coords current_tile,
	achilles_slam::coords previous_tile,
	std::uint16 width, 
	std::vector<achilles_slam::coords> invalid)
{
	tile = current_tile;
	if(current_tile.y != 0 &&
		current_tile.y - 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coords left;
		left.x = current_tile.x;
		left.y = current_tile.y - 1;
		neighbours.push(left);
	}

	if (current_tile.x != 0 &&
		current_tile.x - 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coords top;
		top.x = current_tile.x - 1;
		top.y = current_tile.y;
		neighbours.push(top);
	}

	if (current_tile.y != width -1 &&
		current_tile.y + 1 != previous_tile.y /*&& Check if invalid has this node*/)
	{
		achilles_slam::coords right;
		right.x = current_tile.x;
		right.y = current_tile.y + 1;
		neighbours.push(right);
	}

	if (current_tile.x != width -1 &&
		current_tile.x + 1 != previous_tile.x /*&& CHeck if invalid has this node*/)
	{
		achilles_slam::coords bottom;
		bottom.x = current_tile.x + 1;
		bottom.y = current_tile.y;
		neighbours.push(bottom);
	}
}
	
achilles_slam::coords path_plan_helper::get_next_neighbour()
{
	achilles_slam::coords next = neighbours.pop();
	return next;
}