#pragma once

#include "data.h"
#include "control_points.h"
#include <iostream>

class MissingConnectingPoints: public ControlPointsMiner {
public:
	std::vector<ControlPointsMiner::ControlPoint*> findControlPoints(Data::Part* ref, Data::Part* target);
};
