#include <stack>

namespace operations {

	enum mission
	{
		candle = 6,
		food = 5,
		mansion = 3,
		cabin = 4,
	};

	std::stack<int> missions;

	void initialize();
	void traverse_to_empty();
	void traverse_to_objective(int curr_mission);
	void grid_traverse();
	void objective_tasks();
	void mission_people();
	void mission_food();
	void mission_candle();
	bool object_mapped(int object);
}