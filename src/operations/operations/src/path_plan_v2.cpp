// #include <deque>
#include "path_plan_v2.h"
#include "ros/ros.h"
#include "achilles_slam/course_map.h"

using namespace path_plan;

bool path_plan::find_path(uint8_t current, uint8_t target, achilles_slam::course_map* map, std::vector<bool>* visited, std::deque <uint8_t>* path)
{
	// Reached end of the path and curr tile is target
	if (current == target)
	{
		path->push_front(current);
		return true;
	}

	// Set tile visited
	(*visited)[current] = true;

	// Get adjacent tiles
	std::vector<uint8_t> adjacent_tiles = get_adjacent(current);

	// Recurse through adjacent tiles
	for (std::vector<uint8_t>::iterator it = adjacent_tiles.begin() ; it != adjacent_tiles.end() ; it++)
	{
		if (*it == target || map->map[*it].target < achilles_slam::tile::TARGET_OCCUPIED)
		{
			// Only check if tile is unvisited and flat
			if (!(*visited)[*it] && map->map[*it].terrain == achilles_slam::tile::TERRAIN_FLAT)
			{
				// Recurse through tile
				if (find_path(*it, target, map, visited, path))
				{
					path->push_front(current);
					return true;
				}
			}
		}
			
	}
	return false;

}

std::deque <uint8_t> path_plan::get_path_to_target(uint8_t target, uint8_t current, achilles_slam::course_map* map)
{
	std::vector<bool> visited(36, false);
	std::deque <uint8_t> path;

	if (find_path(current, target, map, &visited, &path))
	{
		return path;
	}
	else
	{
		ROS_INFO("CANNOT FIND PATH TO TARGET");
	}
}

std::vector<uint8_t> path_plan::get_adjacent(uint8_t tile_num)
{
	uint8_t width = 6;
	std::vector<uint8_t> returnVals;
	if (tile_num%width != 0)
	{
		returnVals.push_back(tile_num - 1);
	}

	if (tile_num%width != 5)
	{
		returnVals.push_back(tile_num + 1);
	}

	if (tile_num/width != 0)
	{
		returnVals.push_back(tile_num - 6);
	}

	if (tile_num/width != 5)
	{
		returnVals.push_back(tile_num + 6);
	}

	return returnVals;
}
