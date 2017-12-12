#pragma once

#include <string>
#include "data.h"
#include "control_points.h"

class MixMatch {
public:
	virtual Data* mix(Data* data1, Data* data2) = 0;
	virtual Data::Part* mixPart(Data::Part* part1, Data::Part* part2) = 0;
	void scaleBoundingBox(Data::Part* from, Data::Part* to);

public:
	std::string name;
	std::vector<ControlPointsMiner::ControlPoint*> totalControlPoints;
};
