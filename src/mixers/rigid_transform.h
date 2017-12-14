#pragma once

#include "data.h"
#include "mix_match.h"
#include "control_points.h"
#include "utils.h"
#include <iostream>
#include <vector>

class RigidTransform: public MixMatch {
public:
	RigidTransform(std::vector<ControlPointsMiner*> miners);
	Data* mix(Data* chair1, Data* chair2);
	Data::Part* mixPart(Data::Part* ref, Data::Part* target);

private:
	std::vector<ControlPointsMiner*> miners;
};
