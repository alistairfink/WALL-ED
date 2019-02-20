#include <stack>
#include "../include/operations.h"

using namespace operations;

int main()
{
	initialize();
	while(!missions.empty())
	{
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
	return 1;
}

void initialize()
{
	missions.push(mission::candle);
	missions.push(mission::food);
	missions.push(mission::people);
	missions.push(mission::people);
}

void traverse_to_empty()
{
	// Path plan
	// Check if objective is mapped at each step.
}

void traverse_to_objective()
{
	// Call path plan to mapped objective 
	// When there call objective_tasks()
}

void grid_traverse()
{
	// If detected then -> objective_tasks()
}

void objective_tasks()
{
	int curr = missions.top();
	switch (curr)
	{
		case mission::candle:
			mission_candle();
			break;
		case mission::food:
			mission_food();
			break;
		case mission::people:
			mission_people();
			break;
		default:
			throw std::exception();
			// Idk. Shouldn't get here. If it does somethings fucked
	}
	// Pop stack. Assumes mission was successful. Idk if we want more oversight here
}

void mission_people()
{
	// Do thing for people
}

void mission_food()
{
	// Do thing for food
}

void mission_candle()
{
	// Do thing for candle
}