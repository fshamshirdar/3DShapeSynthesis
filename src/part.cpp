#include "data.h"

Data::Part::Part()
{
	Eigen::Vector3f min, max;
	min << 100.0, 100.0, 100.0;
	max << -100.0, -100.0, -100.0;
	boundingBox = Eigen::AlignedBox3f(min, max);
}
