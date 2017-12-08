#include "mix_match.h"
#include "mixers/simple_box.h"

Data* SimpleBox::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* back = chair2->findPartByType(Data::Part::BACK_SHEET);
	output->replacePartByType(back);

	return output;
}
