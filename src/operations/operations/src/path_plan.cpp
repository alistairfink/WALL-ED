#include "ros/ros.h"
#include "./path_plan.h"

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

path_plan_helper::path_plan_helper(achilles_slam::coords current_tile, std::vector<achilles_slam::coords> invalid)
{

}
	
achilles_slam::coords path_plan_helper::get_next_neighbour()
{
	
}