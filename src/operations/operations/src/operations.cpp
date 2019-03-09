#include <stack>
#include "ros/ros.h"
#include "operations/operations.h"
#include "./path_plan.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"

void operations::initialize()
{
	operations::missions.push(operations::candle);
	operations::missions.push(operations::food);
	operations::missions.push(operations::mansion);
	operations::missions.push(operations::cabin);
}

void operations::traverse_to_empty(
	int curr_mission, 
	achilles_slam::course_map map, 
	achilles_slam::coord curr_pos)
{
	// how to find empty tile?
	achilles_slam::coord dest;
	// how to get invalid?
	std::vector<achilles_slam::coord> invalid;
	std::deque<achilles_slam::coord> path_plan = path_plan::path_plan_objective(map, curr_pos, dest, invalid);

	while (!path_plan.empty())
	{
		achilles_slam::coord curr = path_plan.front();
		// Do stuff to move to tile here
		// Check if objective is mapped at each step.
		path_plan.pop_front();
	}
}

void operations::traverse_to_objective(
	int curr_mission, 
	achilles_slam::course_map map, 
	achilles_slam::coord curr_pos, 
	achilles_slam::coord dest)
{
	// how to get invalid?
	std::vector<achilles_slam::coord> invalid;
	std::deque<achilles_slam::coord> path_plan = path_plan::path_plan_objective(map, curr_pos, dest, invalid);

	while (path_plan.back() != path_plan.front())
	{
		achilles_slam::coord curr = path_plan.front();
		// Nav to next tile
		path_plan.pop_front();
	}

	operations::objective_tasks();
}

void operations::grid_traverse(
	achilles_slam::course_map map,
	achilles_slam::coord curr_pos)
{
	// how to get invalid?
	std::vector<achilles_slam::coord> invalid = null;
	std::deque<achilles_slam::coord> path_plan = path_plan::path_plan_objective(map, curr_pos, invalid);

	while (path_plan.back() != path_plan.front())
	{
		achilles_slam::coord curr = path_plan.front();
		// Nav to next tile
		path_plan.pop_front();
	}

	operations::objective_tasks();
}

void operations::objective_tasks()
{
	int curr = operations::missions.top();
	switch (curr)
	{
		case operations::candle:
			operations::mission_candle();
			break;
		case operations::food:
			operations::mission_food();
			break;
		case operations::mansion:
			operations::mission_people();
			break;
		case operations::cabin:
			operations::mission_people();
			break;
		default:
			throw std::exception();
			// Should never  get here. If it does somethings very wrong
	}

	operations::missions.pop();
	// Pop stack. Assumes mission was successful. Idk if we want more oversight here
}

void operations::mission_people()
{
	// Do thing for people
}

void operations::mission_food()
{
	// Get sensor data
	// Check sensor data
	// complete?
}

void operations::mission_candle()
{
	// Do thing for candle
}

achilles_slam::coord operations::object_mapped(int object, achilles_slam::course_map map)
{
	for (int i = 0; i < map.map.size(); i++)
	{
		if (map.map[i].target == object)
		{
			std::uint16 x = i/map.width;
			std::uint16 y = i%map.width;
			achilles_slam::coord ret_val;
			ret_val.x = x;
			ret_val.y = y;
			return ret_val;
		}
	}

	return NULL;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "operations");

	operations::initialize();
	while(!operations::missions.empty())
	{
		// Get map
		achilles_slam::coord pos = operations::object_mapped(operations::missions.top(), );
		if (pos != NULL)
	 	{
			operations::traverse_to_objective(operations::missions.top());
		}
		else
		{
			if (operations::missions.top() == operations::food)
			{

			}
			else
			{
				// how to know where to go?
			}
		}
		// 1. Is object mapped?
		// yes?
		// 		2. Path plan to objective -> traverse_to_objective()
		// no?
		// 		2. Is curr mission food?
		//		yes?
		//			4. Grid Traversal -> grid_traverse()
		//		no?
		//			3. Path plan to empty -> traverse_to_empty()
	}
	// Traverse to start
	return 0;
}