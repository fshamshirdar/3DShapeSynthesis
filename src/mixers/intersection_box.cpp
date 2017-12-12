#include "mix_match.h"
#include "mixers/intersection_box.h"

Data* IntersectionBox::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);
	//Data::Part* target = chair1->findPartByType(Data::Part::FOUR_LEGGED);
	//Data::Part* ref = chair2->findPartByType(Data::Part::FOUR_LEGGED);
	
	mixPart(ref, target);
	output->replacePartByType(ref);

	return output;
}

Data::Part* IntersectionBox::mixPart(Data::Part* ref, Data::Part* target)
{
	ref->findNeighborsByBoxIntersection();
	target->findNeighborsByBoxIntersection();

	Eigen::Vector3f scales = Eigen::Vector3f::Ones();
	Eigen::Vector3f translations = Eigen::Vector3f::Zeros();
	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
			if ((*rnit)->type == (*tnit)->type) {
				PartIntersection* refNeighbor = (*rnit);
				PartIntersection* targetNeighbor = (*tnit);

				Eigen::Vector3f to = (box.max() - box.min());
				Eigen::Vector3f from = (boundingBox.max() - boundingBox.min());
				Eigen::Vector3f scale;
				scale[0] = to[0] / from[0];
				scale[1] = to[1] / from[1];
				scale[2] = to[2] / from[2];

				
			}
		}
	}


	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (target->boundingBox.min() - ref->boundingBox.min());
	ref->translate(translation);

	return ref;
}
