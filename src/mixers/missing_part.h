#pragma once

#include "data.h"
#include "mix_match.h"

class MissingPart: public MixMatch {
public:
	MissingPart();
	Data* mix(Data* chair1, Data* chair2);
	Data::Part* mixPart(Data::Part* part1, Data::Part* part2);
};
