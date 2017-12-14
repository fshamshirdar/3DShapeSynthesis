#ifndef __SIMPLE_BOX_H__
#define __SIMPLE_BOX_H__

#include "data.h"
#include "mix_match.h"

class SimpleBox: public MixMatch {
public:
	SimpleBox();
	Data* mix(Data* chair1, Data* chair2);
	Data::Part* mixPart(Data::Part* part1, Data::Part* part2);
};

#endif
