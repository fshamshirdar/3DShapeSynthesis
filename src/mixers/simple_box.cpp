#include "mix_match.h"
#include "mixers/simple_box.h"

Data* SimpleBox::mix(Data* chair1, Data* chair2)
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

Data::Part* SimpleBox::mixPart(Data::Part* ref, Data::Part* target)
{
	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (target->boundingBox.min() - ref->boundingBox.min());
	ref->translate(translation);

	return ref;
}
