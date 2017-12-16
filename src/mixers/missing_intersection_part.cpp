#include "mix_match.h"
#include "mixers/missing_intersection_part.h"
#include <iostream>

#define DIST_THR 10.
#define MAX_SCALE_THR 2.0
#define MIN_SCALE_THR 0.5

MissingIntersectionPart::MissingIntersectionPart()
{
	this->name = "missing intersection part";
}

Data* MissingIntersectionPart::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	Data::Part* target = chair1->findPartByType(Data::Part::SEAT_SHEET); // dummy
	Data::Part* ref = chair2->findPartByType(Data::Part::LEFT_HANDLE);

	mixPart(ref, target);
	output->replacePartByType(ref);

	return output;
}

Data::Part* MissingIntersectionPart::mixPart(Data::Part* ref, Data::Part* target)
{
	ref->findNeighborsByBoxIntersection();
	Data* targetChair = target->parent;

	int len = 0;
	Eigen::Vector3f scales = Eigen::Vector3f::Zero();
	Eigen::Vector3f translations = Eigen::Vector3f::Zero();
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		Data::PartIntersection* refIntersection = (*rnit);
		Data::Part* refNeighbor = refIntersection->neighbor;

		Data::Part* targetPair = targetChair->findPartByType(refNeighbor->type);
		if (targetPair) {
			Eigen::Vector3f from = (refNeighbor->boundingBox.max() - refNeighbor->boundingBox.min());
			Eigen::Vector3f to = (targetPair->boundingBox.max() - targetPair->boundingBox.min());
			Eigen::Vector3f scale;
			scale[0] = to[0] / from[0];
			scale[1] = to[1] / from[1];
			scale[2] = 1.; // to[2] / from[2];
			scales += scale;

			Eigen::Vector3f translation = (targetPair->boundingBox.min() + targetPair->boundingBox.max()) / 2. -
				(refNeighbor->boundingBox.min() + refNeighbor->boundingBox.max()) / 2.;
			translation[2] = targetPair->boundingBox.min()[2] - refNeighbor->boundingBox.min()[2];
			// translation[2] = (target->boundingBox.max()[2] + target->boundingBox.min()[2]) / 2. - (ref->boundingBox.max()[2] + ref->boundingBox.min()[2]) / 2.; // to[2] / from[2];
			translations += translation;

			len ++;
		}
	}

	if (len == 0) {
		scales = Eigen::Vector3f::Ones();
		translations = Eigen::Vector3f::Zero();
		return NULL;
	} else {
		scales[0] /= len; scales[1] /= len; scales[2] /= len;
		translations[0] /= len; translations[1] /= len; translations[2] /= len;
	}

	if (scales[0] < MIN_SCALE_THR || scales[1] < MIN_SCALE_THR || scales[2] < MIN_SCALE_THR ||
	    scales[0] > MAX_SCALE_THR || scales[1] > MAX_SCALE_THR || scales[2] > MAX_SCALE_THR) {
		return NULL;
	}

	std::cout << "len: " << len << std::endl << scales << std::endl;

	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(scales, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	ref->translate(translations);

	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		Data::PartIntersection* refIntersection = (*rnit);
		Data::Part* refNeighbor = refIntersection->neighbor;

		Data::Part* targetPair = targetChair->findPartByType(refNeighbor->type);
		if (targetPair) {
//			Eigen::Vector3f baseScale = (refNeighbor->boundingBox.min() + refNeighbor->boundingBox.max()) / 2.;
//			refNeighbor->scale(target->boundingBox, baseScale);
//			// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
//			Eigen::Vector3f translation = (targetPair->boundingBox.min() - refNeighbor->boundingBox.min());
//			refNeighbor->translate(translation);
			refNeighbor->scale(scales, baseScale);
			refNeighbor->translate(translations);

			std::vector<ControlPointsMiner::ControlPoint*> controlPoints;
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

					totalControlPoints.push_back(pair);
				}
			}

			// (*vrnit)->pos = closestVertex->pos;
			ControlPointsMiner::ControlPoint* closestControlPoint = NULL;
			float minDist = 1000.;
			for (auto vrmnit=(*rnit)->myVertices.begin(); vrmnit!=(*rnit)->myVertices.end(); vrmnit++) {
				for (auto cit=controlPoints.begin(); cit!=controlPoints.end(); cit++) {
					float dist = ((*cit)->vertex->pos - (*vrmnit)->pos).norm();
					if (dist < minDist) {
						minDist = dist;
						closestControlPoint = (*cit);
					}
				}
				if (closestControlPoint) {
					(*vrmnit)->pos += closestControlPoint->translation;
				}
			}
		}
	}

	return ref;
}
