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
	std::deque<path_plan::path_plan_helper> helper;
	path_plan::path_plan_helper current(start, start, map.width, invalid);
	helper.push_back(current);

	while(helper.back().tile.x != dest.x && helper.back().tile.y != dest.y)
	{
		if (helper.back().neighbours.empty())
		{
			helper.pop_back();
		}

		path_plan::path_plan_helper next(helper.back().get_next_neighbour(), helper.back().tile, map.width, invalid);
		helper.push_back(next);
	}

	std::deque<achilles_slam::coord> path_plan;
	while (helper.size() > 0)
	{
		path_plan::path_plan_helper curr = helper.front();
		helper.pop_front();
		path_plan.push_back(curr.tile);
	}

	return path_plan;
}

std::deque<achilles_slam::coord> path_plan::path_plan_grid(
	achilles_slam::course_map map, 
	achilles_slam::coord start, 
	std::vector<achilles_slam::coord> invalid)
{
	int length = map.width * map.width - invalid.size();
	std::deque<path_plan::path_plan_helper> helper;
	path_plan::path_plan_helper current(start, start, map.width, invalid);
	helper.push_back(current);

	while (helper.size() < length)
	{
		if (helper.back().neighbours.empty())
		{
			helper.pop_back();
		}

		path_plan::path_plan_helper next(helper.back().get_next_neighbour(), helper.back().tile, map.width, invalid);
		helper.push_back(next);
	}

	std::deque<achilles_slam::coord> path_plan;
	while (helper.size() > 0)
	{
		path_plan::path_plan_helper curr = helper.front();
		helper.pop_front();
		path_plan.push_back(curr.tile);
	}

	return path_plan;
}

path_plan_helper::path_plan_helper(
	achilles_slam::coord current_tile,
	achilles_slam::coord previous_tile,
	uint16_t width, 
	std::vector<achilles_slam::coord> invalid)
{
	// May have to redo if we have a sense of weighting
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
	if (!neighbours.empty())
	{
		achilles_slam::coord next = neighbours.top();
		neighbours.pop();
		return next;
	}
}