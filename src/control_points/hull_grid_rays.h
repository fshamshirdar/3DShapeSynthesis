#pragma once

#include "data.h"
#include "control_points.h"
#include <iostream>

class HullGridRays: public ControlPointsMiner {
public:
	class HullGridPoint {
	public:
		Data::Vertex* vertex;
		float dist;
	public:
		HullGridPoint() {
			vertex = NULL;
			dist = 1000.0;
		}
	};

public:
	HullGridRays(int nx, int ny, int nz);
	std::vector<ControlPointsMiner::ControlPoint*> findControlPoints(Data::Part* ref, Data::Part* target);
	std::vector<ControlPointsMiner::ControlPoint*> findCorrespondingPoints(HullGridRays::HullGridPoint*** refPoints, HullGridRays::HullGridPoint*** targetPoints, int n, int m);
	HullGridPoint*** findXZHullPoints(Data::Part* part, int n, int m);
	HullGridPoint*** findXYHullPoints(Data::Part* part, int n, int m);	
	HullGridPoint*** findYZHullPoints(Data::Part* part, int n, int m);
	bool RayIntersectsTriangle(Eigen::Vector3f rayOrigin, 
                           Eigen::Vector3f rayTarget, 
                           Data::Face* face,
                           Eigen::Vector3f& outIntersectionPoint);

private:
	int nx, ny, nz;
};
