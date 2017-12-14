#pragma once

#include <string>
#include "data.h"
#include "control_points.h"

class MixMatch {
public:
	virtual Data::Part* mixPart(Data::Part* part1, Data::Part* part2) { return NULL; };
	void scaleBoundingBox(Data::Part* from, Data::Part* to);

	Data* mix();
	Data* mixBack(Data* data1, Data* data2);
	Data* mixHandles(Data* data1, Data* data2);
	Data* mixLeg(Data* data1, Data* data2);

public:
	std::string name;
	std::vector<ControlPointsMiner::ControlPoint*> totalControlPoints;
	std::vector<Data*> datas;
};
