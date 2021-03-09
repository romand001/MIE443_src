#pragma once
#include <vector>

class Navigation {
	public:
		static bool moveToGoal(float xGoal, float yGoal, float phiGoal);
        void visitGoals(std::vector<std::vector<float>> path);
};
