#ifndef __SIMPLE_BOX_H__
#define __SIMPLE_BOX_H__

#include "data.h"
#include "mix_match.h"

class SimpleBox: public MixMatch {
public:
	Data* mix(Data* chair1, Data* chair2);
};

#endif
