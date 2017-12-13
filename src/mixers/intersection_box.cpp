#include "mix_match.h"
#include "mixers/intersection_box.h"
#include <iostream>

Data* IntersectionBox::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	//Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	//Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* target = chair1->findPartByType(Data::Part::LEG);
	Data::Part* ref = chair2->findPartByType(Data::Part::LEG);
	
	mixPart(ref, target);
	output->replacePartByType(ref);

	return output;
}

Data::Part* IntersectionBox::mixPart(Data::Part* ref, Data::Part* target)
{
	ref->findNeighborsByBoxIntersection();
	target->findNeighborsByBoxIntersection();

	int len = 0;
	Eigen::Vector3f scales = Eigen::Vector3f::Zero();
	Eigen::Vector3f translations = Eigen::Vector3f::Zero();
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
			if ((*rnit)->neighbor->type == (*tnit)->neighbor->type) {
				Data::PartIntersection* refNeighbor = (*rnit);
				Data::PartIntersection* targetNeighbor = (*tnit);

				Eigen::Vector3f from = (refNeighbor->boundingBox.max() - refNeighbor->boundingBox.min());
				Eigen::Vector3f to = (targetNeighbor->boundingBox.max() - targetNeighbor->boundingBox.min());
				Eigen::Vector3f scale;
				scale[0] = to[0] / from[0];
				scale[1] = to[1] / from[1];
				scale[2] = 1.; // to[2] / from[2];
				scales += scale;

				Eigen::Vector3f translation = (targetNeighbor->boundingBox.min() - refNeighbor->boundingBox.min());
				translations += translation;

				len ++;
			}
		}
	}

	if (len == 0) {
		scales = Eigen::Vector3f::Ones();
		translations = Eigen::Vector3f::Zero();
	} else {
		scales[0] /= len; scales[1] /= len; scales[2] /= len;
		translations[0] /= len; translations[0] /= len; translations[0] /= len;
	}

	std::cout << scales << std::endl;

	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(scales, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	ref->translate(translations);

	return ref;
}
