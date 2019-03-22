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
#include "sensor_package/AddTwoInts.h"
#include "path_plan_v2.h"

void operations::initialize(achilles_slam::course_map map, int start)
{
	int offset = 0;
	int start_pos = 23;//map.width*map.width - 1 - offset;
	direction = 0;
	operations::start->x = start_pos/map.width;
	operations::start->y = start_pos%map.width;
	//operations::missions.push(operations::food);
	operations::missions.push(operations::candle);
	operations::missions.push(operations::mansion);
	operations::missions.push(operations::cabin);

	if (start == 1)
	{
		operations::sand_index.push(28);
		operations::sand_index.push(15);
	}
	else if (start == 2)
	{
		operations::sand_index.push(10);
		operations::sand_index.push(14);
	}
	else if (start == 3)
	{
		operations::sand_index.push(28);
		operations::sand_index.push(20);
	}
	else if (start == 4)
	{
		operations::sand_index.push(21);
		operations::sand_index.push(10);
	}
}

void operations::traverse_to_empty(
	int curr_mission, 
	achilles_slam::course_map map)
{
	achilles_slam::coord dest = get_closest_unvisited(map);
	achilles_slam::coord curr_pos = map.robot_pos;
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, &dest);
	std::deque<achilles_slam::coord> path = path_plan::path_plan_objective(map, curr_pos, dest, invalid);
	path.pop_front();
	achilles_slam::coord curr = curr_pos;

	while (!path.size() > 0)
	{
		achilles_slam::coord next = path.front();
		ROS_INFO("CURRENT: X %i Y %i, NEXT: X %i Y %i", curr.x, curr.y, next.x, next.y);
		operations::turn_properly(curr, next);

		movement::straight(movement::NOMINAL, movement::TILE_DIST, operations::motor);
		operations::update_tile(next, map);
		path.pop_front();
		curr = next;
	}
}

void operations::traverse_to_objective(
	achilles_slam::course_map map, 
	achilles_slam::coord* dest)
{
	achilles_slam::coord curr_pos = map.robot_pos;
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, dest);
	std::deque<uint8_t> path = path_plan::get_path_to_target(6*curr_pos.x+curr_pos.y, 6*dest->x+dest->y, &map);
	path.pop_front();
	achilles_slam::coord curr = curr_pos;

	while (path.front() != 6*dest->x + dest->y)
	{
		achilles_slam::coord next;
		next.x = path.front()/map.width;
		next.y = path.front()&map.width;
		ROS_INFO("CURRENT: X %i Y %i, NEXT: X %i Y %i", curr.x, curr.y, next.x, next.y);
		operations::turn_properly(curr, next);

		movement::straight(movement::NOMINAL, movement::TILE_DIST, operations::motor);
		operations::update_tile(next, map);
		path.pop_front();
		curr = next;
	}

	operations::objective_tasks();
}

void operations::grid_traverse(achilles_slam::course_map map)
{
	std::vector<achilles_slam::coord> invalid = operations::get_invalid(map, NULL);

	bool found = false;

	while (operations::sand_index.size() > 0 && !found)
	{
		achilles_slam::coord curr_pos = map.robot_pos;
		achilles_slam::coord curr = curr_pos;
		int target_index = operations::sand_index.top();
		achilles_slam::coord dest;
		dest.x = target_index/map.width;
		dest.y = target_index%map.width;
		std::deque<achilles_slam::coord> path = path_plan::path_plan_objective(map, curr_pos, dest, invalid);
		while (path.back().x != path.front().x && path.back().y != path.front().y)
		{
			achilles_slam::coord next = path.front();
			
			int direction_to_go;
			if (curr.x < next.x)
			{
				direction_to_go = DIR_NORTH;
			}
			else if (curr.x > next.x)
			{
				direction_to_go = DIR_SOUTH;
			}
			else if (curr.y < next.y)
			{
				direction_to_go = DIR_EAST;
			}
			else if (curr.y > next.y)
			{
				direction_to_go = DIR_WEST;
			}

			while (direction != direction_to_go)
			{
				if (direction > direction_to_go || (direction == DIR_NORTH && direction_to_go == DIR_WEST))
				{
					int next_dir = direction--;
					if (next_dir < 0)
					{
						next_dir = DIR_WEST;
					}

					movement::turn(movement::LEFT, next_dir, operations::motor);
					direction--;
				}
				else if (direction < direction_to_go || (direction == DIR_WEST && direction_to_go == DIR_NORTH))
				{
					int next_dir = direction++;
					if (next_dir > 3)
					{
						next_dir = DIR_NORTH;
					}

					movement::turn(movement::RIGHT, next_dir, operations::motor);
					direction++;
				}

				if (direction < 0)
				{
					direction = DIR_WEST;
				}
				else if (direction > 3)
				{
					direction = DIR_NORTH;
				}
			}

			movement::straight(movement::NOMINAL, movement::TILE_DIST, operations::motor);
			operations::update_tile(next, map);
			path.pop_front();
			curr = next;
		}

		operations::sand_index.pop();
		movement::straight(movement::NOMINAL * -1, movement::TILE_DIST/2, operations::motor);
		if (1 /*Check Hall Effect*/)
		{
			// Turn LED on
			ros::Duration(1).sleep();
			found = true;
		}
		movement::straight(movement::NOMINAL, movement::TILE_DIST/2, operations::motor);
	}

	operations::missions.pop();
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
		case operations::food:
			operations::mission_food();
			break;
		case operations::candle:
			//operations::mission_candle();
			//break;
		case operations::mansion:
			//operations::mission_people();
			//break;
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
	achilles_slam::course_map map = operations::get_map();
	achilles_slam::coord* obj_pos = operations::object_mapped(operations::missions.top(), map);
	operations::turn_properly(map.robot_pos, *obj_pos);
	operations::use_controls(2);
	ros::Duration(30).sleep();
	operations::use_controls(3);
	operations::use_controls(4);
	ros::Duration(3).sleep();
	operations::use_controls(5);
	delete obj_pos;
	obj_pos = NULL;
}

void operations::mission_food()
{
	// lol
	// Get sensor data
	// Check sensor data
	// complete?
}

void operations::mission_candle()
{
	// lmao
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
		if (x != curr.x && y != curr.y && !map.map[i].visited)
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
	achilles_slam::course_map map = map_service.response.silicon_valley;
	int tempx = map.robot_pos.x;
	map.robot_pos.x = map.robot_pos.y;
	map.robot_pos.y = tempx;
	return map;
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

void operations::turn_properly(achilles_slam::coord curr, achilles_slam::coord next)
{
	int direction_to_go;
	if (curr.x < next.x)
	{
		direction_to_go = DIR_NORTH;
	}
	else if (curr.x > next.x)
	{
		direction_to_go = DIR_SOUTH;
	}
	else if (curr.y < next.y)
	{
		direction_to_go = DIR_EAST;
	}
	else if (curr.y > next.y)
	{
		direction_to_go = DIR_WEST;
	}

	if (operations::direction > direction_to_go || (operations::direction == DIR_NORTH && direction_to_go == DIR_WEST))
	{
		movement::turn(movement::RIGHT, direction_to_go, operations::motor);
	}
	else if (operations::direction < direction_to_go || (operations::direction == DIR_WEST && direction_to_go == DIR_NORTH))
	{
		movement::turn(movement::LEFT, direction_to_go, operations::motor);
	}

	operations::direction = direction_to_go;
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

	for (int i = 0; i < map.bad_zones.size(); i++)
	{
		int x = map.bad_zones[i]/map.width;
		int y = map.bad_zones[i]%map.width;
		achilles_slam::coord temp;
		temp.x = x;
		temp.y = y;
		invalid.push_back(temp);
	}

	return invalid;	
}

int64_t operations::use_controls(int64_t value)
{
	sensor_package::AddTwoInts service;
	service.request.input = value;
	while (!controls_client.call(service));
    if (value == 1)
    {
		return service.response.hall;
    }
    else if(value == 2)
    {
        return service.response.fanOn;
    }    
    else if (value == 3)
    {
        return service.response.fanOff;
    }    
    else if (value == 4)
    {
        return service.response.LEDOn;
    }    
    else
    {
        return service.response.LEDOff;
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "operations");
  	ros::NodeHandle n;
  	n.param<int>("/start_position", operations::starting_tile, 1);
	operations::map_client = n.serviceClient<achilles_slam::get_course_map>("get_course_map");
	operations::update_map_client = n.serviceClient<achilles_slam::update_course_map>("update_course_map");
	operations::controls_client = n.serviceClient<sensor_package::AddTwoInts>("AddTwoInts");

	ros::Subscriber laser = n.subscribe("/scan", 1, movement::get_lidar);
	ros::Subscriber orientation = n.subscribe("/slam_out_pose", 1, movement::orientation);
	operations::motor = new motor_abs::motor_driver("/dev/ttyACM0", 115200);

	/*operations::direction = operations::DIR_WEST;
	achilles_slam::coord current;
	current.x = 3;
	current.y = 3;
	achilles_slam::coord next;
	next.x = 4;
	next.y = 3;
	operations::turn_properly(current, next);*/
	//ros::spinOnce();
	// Straight Test
	//movement::straight(movement::NOMINAL, movement::TILE_DIST, operations::motor);

	// Turn Test
	/*movement::turn(movement::LEFT, operations::DIR_WEST, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::LEFT, operations::DIR_SOUTH, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::LEFT, operations::DIR_EAST, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::RIGHT, operations::DIR_SOUTH, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::RIGHT, operations::DIR_WEST, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::RIGHT, operations::DIR_NORTH, operations::motor);
	ros::Duration(1).sleep();
	movement::turn(movement::RIGHT, operations::DIR_EAST, operations::motor);*/

	operations::direction = operations::DIR_WEST;
	achilles_slam::course_map map = operations::get_map();
	int tempx = map.robot_pos.x;
	map.robot_pos.x = map.robot_pos.y;
	map.robot_pos.y = tempx;
	int goal = 7;
	achilles_slam::coord* dest = new achilles_slam::coord;
	dest->x = goal/map.width;
	dest->y = goal%map.width;
	ROS_INFO("Done Setup");
	operations::traverse_to_objective(map, dest);

/*
	achilles_slam::course_map orig_map = operations::get_map();
	operations::initialize(orig_map, operations::starting_tile);

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
	operations::traverse_to_objective(map, operations::start);
	return 0;*/
}
