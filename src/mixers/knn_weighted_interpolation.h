#pragma once

#include "data.h"
#include "mix_match.h"
#include "control_points.h"
#include <iostream>
#include <vector>

class KNNWeightedInterpolation: public MixMatch {
public:
	KNNWeightedInterpolation(std::vector<ControlPointsMiner*> miners, int K, float maxDist);
	Data* mix(Data* chair1, Data* chair2);
	Data::Part* mixBack(Data::Part* ref, Data::Part* target);

private:
	std::vector<ControlPointsMiner*> miners;
	int K;
	float maxDist;
};
