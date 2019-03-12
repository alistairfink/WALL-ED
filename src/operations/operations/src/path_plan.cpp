#include <stack>
#include <algorithm>
#include "ros/ros.h"
#include "operations/path_plan.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"

using namespace path_plan;

std::vector<achilles_slam::coord> path_plan::get_invalid(achilles_slam::course_map map, std::vector<achilles_slam::coord> other_invalid)
{
	std::vector<achilles_slam::coord> invalid;

	for(int i = 0; i < map.target_list.size(); i ++)
	{
		int target_location = map.target_list[i];
		achilles_slam::coord invalid_pos;
		invalid_pos.x = i/map.width;
		invalid_pos.y = i%map.width;
		invalid.push_back(invalid_pos);
	}

	for (int i = 0; i < other_invalid.size(); i++)
	{
		invalid.push_back(other_invalid[i]);
	}

	return invalid;
}

std::deque<achilles_slam::coord> path_plan::path_plan_objective(
	achilles_slam::course_map map, 
	achilles_slam::coord start, 
	achilles_slam::coord dest, 
	std::vector<achilles_slam::coord> other_invalid)
{
	std::deque<path_plan::path_plan_helper> helper;
	std::vector<achilles_slam::coord> invalid = get_invalid(map, other_invalid);
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
	std::vector<achilles_slam::coord> other_invalid)
{
	std::vector<achilles_slam::coord> invalid = get_invalid(map, other_invalid);
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
		current_tile.y - 1 != previous_tile.y)
	{
		achilles_slam::coord left;
		left.x = current_tile.x;
		left.y = current_tile.y - 1;
		if (!path_plan::find(invalid, left))
		{
			neighbours.push(left);
		}
	}

	if (current_tile.x != 0 &&
		current_tile.x - 1 != previous_tile.y)
	{
		achilles_slam::coord top;
		top.x = current_tile.x - 1;
		top.y = current_tile.y;
		if (!path_plan::find(invalid, top))
		{
			neighbours.push(top);
		}
	}

	if (current_tile.y != width -1 &&
		current_tile.y + 1 != previous_tile.y)
	{
		achilles_slam::coord right;
		right.x = current_tile.x;
		right.y = current_tile.y + 1;
		if (!path_plan::find(invalid, right))
		{
			neighbours.push(right);
		}
	}

	if (current_tile.x != width -1 &&
		current_tile.x + 1 != previous_tile.x)
	{
		achilles_slam::coord bottom;
		bottom.x = current_tile.x + 1;
		bottom.y = current_tile.y;
		if (!path_plan::find(invalid, bottom))
		{
			neighbours.push(bottom);
		}
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

bool path_plan::find(std::vector<achilles_slam::coord> list, achilles_slam::coord find_me)
{
	for (int i = 0; i < list.size(); i++)
	{
		if (list[i].x == find_me.x && list[i].y == find_me.y)
		{
			return true;
		}
	}

	return false;
}