#ifndef __MIX_MATCH_H__
#define __MIX_MATCH_H__

#include <string>
#include "data.h"

class MixMatch {
public:
	virtual Data* mix(Data* data1, Data* data2) = 0;
	void scaleBoundingBox(Data::Part* from, Data::Part* to);

public:
	std::string name;
	std::vector<Data::Vertex*> controlPointVertices;
};

#endif
