#include "mix_match.h"
#include "mixers/simple_box.h"

Data* SimpleBox::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::SEAT_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::SEAT_SHEET);

	scaleBoundingBox(ref, target);
	output->replacePartByType(ref);

	return output;
}
