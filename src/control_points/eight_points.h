#pragma once

#include "data.h"
#include "control_points.h"

class EightPoints: public ControlPointsMiner {
public:
	std::vector<ControlPointsMiner::ControlPoint*> findControlPoints(Data::Part* ref, Data::Part* target);
	Data::Vertex** find8Points(Data::Part* part);
};
