#include "mix_match.h"
#include "mixers/missing_part.h"
#include <iostream>

MissingPart::MissingPart()
{
	this->name = "missing part";
}

Data* MissingPart::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	Data::Part* target = chair1->findPartByType(Data::Part::SEAT_SHEET); // dummy
	Data::Part* ref = chair2->findPartByType(Data::Part::LEFT_HANDLE);

	mixPart(ref, target);
	output->replacePartByType(ref);

	return output;
}

Data::Part* MissingPart::mixPart(Data::Part* ref, Data::Part* target)
{
	ref->findNeighborsByBoxIntersection();
	Data* targetChair = target->parent;

	std::cout << ref->neighbors.size() << std::endl;
	int len = 0;
	Eigen::Vector3f scales = Eigen::Vector3f::Zero();
	Eigen::Vector3f translations = Eigen::Vector3f::Zero();
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		Data::PartIntersection* refNeighbor = (*rnit);
		Data::Part* neighbor = refNeighbor->neighbor;

		Data::Part* targetPair = targetChair->findPartByType(neighbor->type);
		if (targetPair) {
			Eigen::Vector3f from = (neighbor->boundingBox.max() - neighbor->boundingBox.min());
			Eigen::Vector3f to = (targetPair->boundingBox.max() - targetPair->boundingBox.min());
			Eigen::Vector3f scale;
			scale[0] = to[0] / from[0];
			scale[1] = to[1] / from[1];
			scale[2] = 1.; // to[2] / from[2];
			scales += scale;

			Eigen::Vector3f translation = (targetPair->boundingBox.min() + targetPair->boundingBox.max()) / 2. -
						      (neighbor->boundingBox.min() + neighbor->boundingBox.max()) / 2.;
			translation[2] = targetPair->boundingBox.min()[2] - neighbor->boundingBox.min()[2];
			// translation[2] = (target->boundingBox.max()[2] + target->boundingBox.min()[2]) / 2. - (ref->boundingBox.max()[2] + ref->boundingBox.min()[2]) / 2.; // to[2] / from[2];
			translations += translation;

			len ++;
		}
	}

	if (len == 0) {
		scales = Eigen::Vector3f::Ones();
		translations = Eigen::Vector3f::Zero();
	} else {
		scales[0] /= len; scales[1] /= len; scales[2] /= len;
		translations[0] /= len; translations[1] /= len; translations[2] /= len;
	}

	std::cout << "len: " << len << std::endl << scales << std::endl;

	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(scales, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	ref->translate(translations);

	return ref;
}
