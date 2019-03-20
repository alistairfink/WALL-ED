#include <stack>
#include <cstdlib>
#include "ros/ros.h"
#include "operations/operations.h"
#include "operations/path_plan.h"
#include "operations/movement.h"
#include "motor_driver/motor_driver.h"
#include "achilles_slam/coord.h"
#include "achilles_slam/course_map.h"
#include "achilles_slam/get_course_map.h"
#include "achilles_slam/update_course_map.h"

void operations::initialize(achilles_slam::course_map map)
{
	int offset = 0;
	int start_pos = map.width*map.width - 1 - offset;
	start->x = start_pos/map.width;
	start->y = start_pos%map.width;
	operations::missions.push(operations::candle);
	operations::missions.push(operations::food);
	operations::missions.push(operations::mansion);
	operations::missions.push(operations::cabin);
}

void operations::traverse_to_empty(
	int curr_mission, 
	achilles_slam::course_map map)
{
	achilles_slam::coord dest = get_closest_unvisited(map);
	achilles_slam::coord curr_pos = map.robot_pos;
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, &dest);
	std::deque<achilles_slam::coord> path = path_plan::path_plan_objective(map, curr_pos, dest, invalid);

	while (!path.empty())
	{
		achilles_slam::coord curr = path.front();
		// Do stuff to move to tile here
		// Check if objective is mapped at each step.
		path.pop_front();
	}
}

void operations::traverse_to_objective(
	achilles_slam::course_map map, 
	achilles_slam::coord* dest)
{
	achilles_slam::coord curr_pos = map.robot_pos;
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, dest);
	std::deque<achilles_slam::coord> path = path_plan::path_plan_objective(map, curr_pos, *dest, invalid);

	while (path.back().x != path.front().x && path.back().y != path.front().y)
	{
		achilles_slam::coord curr = path.front();
		// orientation?
		// 1. turn
		if (1/*turn left*/)
		{
			movement::turn(movement::LEFT, operations::motor);
		}
		else if (1/* turn right */)
		{
			movement::turn(movement::RIGHT, operations::motor);
		}

		movement::straight(operations::motor);
		operations::update_tile(curr, map);
		path.pop_front();
	}

	operations::objective_tasks();
}

void operations::grid_traverse(achilles_slam::course_map map)
{
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, NULL);
	achilles_slam::coord curr_pos = map.robot_pos;
	std::deque<achilles_slam::coord> path = path_plan::path_plan_grid(map, curr_pos, invalid);

	while (path.back().x != path.front().x && path.back().y != path.front().y)
	{
		achilles_slam::coord curr = path.front();
		// Nav to next tile
		path.pop_front();
	}

	operations::objective_tasks();
}

void operations::objective_tasks()
{
	if (operations::missions.size() <= 0)
	{
		return;
	}

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

achilles_slam::coord operations::get_closest_unvisited(achilles_slam::course_map map)
{
	achilles_slam::coord curr = map.robot_pos;
	achilles_slam::coord closest_unvisited;
	int closeset = 100;
	for (int i  = 0; i < map.map.size(); i++)
	{
		int x = i/map.width;
		int y = i%map.width;
		if (x != curr.x && y != curr.y /* TODO : && unvisited*/)
		{
			if (std::abs(curr.x - x) + std::abs(curr.y - y) < closeset)
			{
				closeset = std::abs(curr.x - x) + std::abs(curr.y - y);
				closest_unvisited.x = x;
				closest_unvisited.y = y;
			} 
		}
	}

	return closest_unvisited;	
}

achilles_slam::coord* operations::object_mapped(int object, achilles_slam::course_map map)
{
	achilles_slam::coord* ret_val = new achilles_slam::coord;
	for (int i = 0; i < map.target_list.size(); i++)
	{
		if (map.map[map.target_list[i]].target == object)
		{
			ret_val->x = map.target_list[i]/map.width;
			ret_val->y = map.target_list[i]%map.width;
			return ret_val;
		}
	}

	return NULL;
}

achilles_slam::course_map operations::get_map()
{
	achilles_slam::get_course_map map_service;
	while (!map_client.call(map_service));
	return map_service.response.silicon_valley;
}

void operations::update_tile(achilles_slam::coord coord, achilles_slam::course_map map)
{
	int index = map.width * coord.x + coord.y;
	achilles_slam::tile tile = map.map[index];
	tile.visited = true;
	achilles_slam::update_course_map map_service;
	map_service.request.tile_coord = index;
	map_service.request.update_tile = tile;
	while (!update_map_client.call(map_service));
}

std::vector<achilles_slam::coord> operations::get_invalid(achilles_slam::course_map map, achilles_slam::coord* dest)
{
	std::vector<achilles_slam::coord> invalid;
	for (int i = 0; i < map.target_list.size(); i++)
	{
		int x = map.target_list[i]/map.width;
		int y = map.target_list[i]%map.width;

		if (dest == NULL || dest->x != x && dest->y != y)
		{
			achilles_slam::coord temp;
			temp.x = x;
			temp.y = y;
			invalid.push_back(temp);
		}
	}

	return invalid;	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "operations");
  	ros::NodeHandle n;
	operations::map_client = n.serviceClient<achilles_slam::get_course_map>("get_course_map");
	operations::update_map_client = n.serviceClient<achilles_slam::update_course_map>("update_course_map");	
	
	ros::Subscriber sub = n.subscribe("/scan", 1, movement::get_lidar);
	operations::motor = new motor_abs::motor_driver("/dev/ttyUSB0", 115200);
	movement::turn(movement::LEFT, operations::motor);
	//movement::init_move(&n);
	ros::spin();

	/*achilles_slam::course_map orig_map = operations::get_map(); 
	operations::initialize(orig_map);

	while(!operations::missions.empty())
	{
		achilles_slam::course_map map = operations::get_map();
		achilles_slam::coord* obj_pos = operations::object_mapped(operations::missions.top(), map);
		if (obj_pos != NULL)
	 	{
			operations::traverse_to_objective(map, obj_pos);
		}
		else
		{
			if (operations::missions.top() == operations::food)
			{
				operations::grid_traverse(map);
			}
			else
			{
				operations::traverse_to_empty(operations::missions.top(), map);
			}
		}

		delete obj_pos;
		obj_pos = NULL;
	}
	
	achilles_slam::course_map map = operations::get_map();
	operations::traverse_to_objective(map, operations::start);*/
	return 0;
}