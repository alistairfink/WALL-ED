#ifndef PATH_PLAN_H
#define PATH_PLAN_H

#include "ros/ros.h"
#include "achilles_slam/course_map.h"
#include <deque>

namespace path_plan 
{

	bool find_path(uint8_t current, uint8_t target, achilles_slam::course_map* map, std::vector<bool>* visited, std::deque <uint8_t>* path);

	std::deque <uint8_t> get_path_to_target(uint8_t target, uint8_t current, achilles_slam::course_map* map);

	std::vector<uint8_t> get_adjacent(uint8_t tile_num);
}

#endif