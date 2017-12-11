#include "mix_match.h"

void MixMatch::scaleBoundingBox(Data::Part* from, Data::Part* to)
{
	from->scale(to->boundingBox);
	from->translate(to->boundingBox.min() - from->boundingBox.min());
}
