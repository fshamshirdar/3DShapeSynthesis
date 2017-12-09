#include "mix_match.h"

void MixMatch::scaleBoundingBox(Data::Part* from, Data::Part* to)
{
	Eigen::Vector3f fromMin = from->boundingBox.min();
	Eigen::Vector3f toMin = to->boundingBox.min();
	Eigen::Vector3f toVec = (to->boundingBox.max() - to->boundingBox.min());
	Eigen::Vector3f fromVec = (from->boundingBox.max() - from->boundingBox.min());
	float xScale = toVec[0] / fromVec[0];
	float yScale = toVec[1] / fromVec[1];
	float zScale = toVec[2] / fromVec[2];

	from->resetBoundingBox();
	for (auto rit = from->regions.begin(); rit != from->regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			
			vertex->pos[0] = (vertex->pos[0] - fromMin[0]) * xScale + toMin[0];
			vertex->pos[1] = (vertex->pos[1] - fromMin[1]) * yScale + toMin[1];
			vertex->pos[2] = (vertex->pos[2] - fromMin[2]) * zScale + toMin[2];
			vertex->pos[3] = 1;

			region->recalculateBoundingBox(vertex);
			from->recalculateBoundingBox(vertex);
		}
	}
}
