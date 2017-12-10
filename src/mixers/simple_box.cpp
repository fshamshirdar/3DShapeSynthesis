#include "mix_match.h"
#include "mixers/simple_box.h"

Data* SimpleBox::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);
	
	Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);

	// scaleBoundingBox(ref, target);
	output->replacePartByType(ref);

	return output;
}
