#include <stack>
#include "ros/ros.h"
#include "operations/operations.h"

void operations::initialize()
{
	missions.push(candle);
	missions.push(food);
	missions.push(people);
	missions.push(people);
}

void operations::traverse_to_empty()
{
	// Path plan
	// Check if objective is mapped at each step.
}

void operations::traverse_to_objective()
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
	int curr = missions.top();
	switch (curr)
	{
		case candle:
			mission_candle();
			break;
		case food:
			mission_food();
			break;
		case people:
			mission_people();
			break;
		default:
			throw std::exception();
			// Idk. Shouldn't get here. If it does somethings fucked
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

bool operations::object_mapped(int object)
{

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "operations");

	operations::initialize();
	while(!operations::missions.empty())
	{
		if (object_mapped(/*top of queue*/))
	 	{
			// Path plan to empty -> traverse_to_empty()
		}
		else
		{
			if ()
			{

			}
			else
			{

			}
		}
		// 1. If object mapped?
		// yes?
		// 		2. Path plan to empty -> traverse_to_empty()
		// no?
		// 		2. Is curr mission food?
		//		yes?
		//			3. Is mapped?
		//			yes?
		//				4. Path plan to food -> traverse_to_objective()
		//			no? 
		//				4. Grid Traversal -> grid_traverse()
		//		no?
		//			3. Path plan to objective -> traverse_to_objective()
	}
	// Traverse to start
	return 0;
}