#pragma once

#include "data.h"
#include "mix_match.h"
#include "control_points.h"

class MissingIntersectionPart: public MixMatch {
public:
	MissingIntersectionPart();
	Data* mix(Data* chair1, Data* chair2);
	Data::Part* mixPart(Data::Part* part1, Data::Part* part2);
};
