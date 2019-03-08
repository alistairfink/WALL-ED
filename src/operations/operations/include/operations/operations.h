#include <stack>

namespace operations {

	enum mission
	{
		candle,
		food,
		people,
	};

	std::stack<int> missions;

	void initialize();
	void traverse_to_empty();
	void traverse_to_objective();
	void grid_traverse();
	void objective_tasks();
	void mission_people();
	void mission_food();
	void mission_candle();
	bool object_mapped(int object);
}