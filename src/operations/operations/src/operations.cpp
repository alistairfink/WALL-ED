#include <stack>
#include "ros/ros.h"
#include "operations/operations.h"

void operations::initialize()
{
	operations::missions.push(operations::candle);
	operations::missions.push(operations::food);
	operations::missions.push(operations::mansion);
	operations::missions.push(operations::cabin);
}

void operations::traverse_to_empty()
{
	// Path plan
	// Check if objective is mapped at each step.
}

void operations::traverse_to_objective(int curr_mission)
{

	// Call path plan to mapped objective 
	// When there call objective_tasks()
}

void operations::grid_traverse()
{
	// If detected then -> objective_tasks()
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
	// Pop stack. Assumes mission was successful. Idk if we want more oversight here
}

void operations::mission_people()
{
	// Do thing for people
}

void operations::mission_food()
{
	// Do thing for food
}

void operations::mission_candle()
{
	// Do thing for candle
}

// May want to return pos here.
bool operations::object_mapped(int object)
{
	// 
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "operations");

	operations::initialize();
	while(!operations::missions.empty())
	{
		// Change to pos
		bool pos = operations::object_mapped(operations::missions.top());
		if (pos /*Check if pos is returned eventually*/)
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