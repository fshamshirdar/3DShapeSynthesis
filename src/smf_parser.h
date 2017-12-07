#ifndef __SMF_PARSER_H__
#define __SMF_PARSER_H__

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <set>
#include "data.h"

class SMFParser {
public:
	SMFParser();
	~SMFParser();
	Data* load(const std::string &filename);
};

#endif
