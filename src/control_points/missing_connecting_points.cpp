#include "mix_match.h"
#include "control_points/missing_connecting_points.h"
#include <set>

#include <iostream>

#define DIST_THR 10.

std::vector<ControlPointsMiner::ControlPoint*> MissingConnectingPoints::findControlPoints(Data::Part* ref, Data::Part* target)
{
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

	Data* targetChair = target->parent;

	ref->findNeighborsByBoxIntersection();

	// scaleToTarget(ref, target);
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		Data::PartIntersection* refIntersection = (*rnit);
		Data::Part* refNeighbor = refIntersection->neighbor;

		Data::Part* targetPair = targetChair->findPartByType(refNeighbor->type);
		if (targetPair) {
			for (auto vrnit=(*rnit)->vertices.begin(); vrnit!=(*rnit)->vertices.end(); vrnit++) {
				Data::Vertex* closestVertex = NULL;
				float minDist = 1000.;
				for (auto rtit=targetPair->regions.begin(); rtit!=targetPair->regions.end(); rtit++) {
					for (auto vrtit=(*rtit)->vertices.begin(); vrtit!=(*rtit)->vertices.end(); vrtit++) {
						float dist = ((*vrtit)->pos - (*vrnit)->pos).norm();
						if (dist < minDist) {
							minDist = dist;
							closestVertex = (*vrtit);
						}
					}
				}
				if (closestVertex && minDist < DIST_THR) {
					ControlPointsMiner::ControlPoint* pair = new ControlPointsMiner::ControlPoint;
					pair->translation = (closestVertex->pos - (*vrnit)->pos);
					pair->vertex = (*vrnit);
					pair->pair = closestVertex;
					pair->dist = minDist;
					controlPoints.push_back(pair);
				}
			}
		}
	}
	// unscaleToRef(ref, target);

	return controlPoints;
}
