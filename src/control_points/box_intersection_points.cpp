#include "mix_match.h"
#include "control_points/box_intersection_points.h"
#include <set>

#include <iostream>

#define DIST_THR 10.

std::vector<ControlPointsMiner::ControlPoint*> BoxIntersectionPoints::findControlPoints(Data::Part* ref, Data::Part* target)
{
	Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	ref->translate(target->boundingBox.min() - ref->boundingBox.min());

	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

	ref->findNeighborsByBoxIntersection();
	target->findNeighborsByBoxIntersection();
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		for (auto vrnit = (*rnit)->vertices.begin(); vrnit != (*rnit)->vertices.end(); vrnit++) {
			Data::Vertex* closestVertex = NULL;
			float minDist = 1000.0;
			for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
				for (auto vtnit = (*tnit)->vertices.begin(); vtnit != (*tnit)->vertices.end(); vtnit++) {
					float dist = ((*vrnit)->pos - (*vtnit)->pos).norm();
					if (dist < minDist) {
						minDist = dist;
						closestVertex = (*vtnit);
					}
				}
			}

			if (minDist < DIST_THR) {
				ControlPointsMiner::ControlPoint* pair = new ControlPointsMiner::ControlPoint;
				pair->translation = (closestVertex->pos - (*vrnit)->pos);
				pair->vertex = (*vrnit);
				pair->pair = closestVertex;
				pair->dist = minDist;
				controlPoints.push_back(pair);
			}
		}
	}

	return controlPoints;
}
