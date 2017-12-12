#include "mix_match.h"
#include "control_points/box_intersection_target_points.h"
#include <set>

#include <iostream>

#define DIST_THR 10.

std::vector<ControlPointsMiner::ControlPoint*> BoxIntersectionTargetPoints::findControlPoints(Data::Part* ref, Data::Part* target)
{
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

	ref->findNeighborsByBoxIntersection();
	target->findNeighborsByBoxIntersection();

	scaleToTarget(ref, target);

	for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
		for (auto vtnit = (*tnit)->vertices.begin(); vtnit != (*tnit)->vertices.end(); vtnit++) {
			Data::Vertex* closestVertex = NULL;
			float minDist = 1000.0;

			for (auto rrit = ref->regions.begin(); rrit != ref->regions.end(); rrit++) {
				for (auto vrrit = (*rrit)->vertices.begin(); vrrit != (*rrit)->vertices.end(); vrrit++) {
					float dist = ((*vtnit)->pos - (*vrrit)->pos).norm();
					if (dist < minDist) {
						minDist = dist;
						closestVertex = (*vrrit);
					}
				}
			}

			if (closestVertex && minDist < DIST_THR) {
				ControlPointsMiner::ControlPoint* pair = new ControlPointsMiner::ControlPoint;
				pair->translation = ((*vtnit)->pos - closestVertex->pos);
				pair->vertex = closestVertex;
				pair->pair = (*vtnit);
				pair->dist = minDist;
				controlPoints.push_back(pair);
			}
		}
	}

	unscaleToRef(ref, target);

	return controlPoints;
}
