#include "data.h"
#include <iostream>
#include <algorithm>

Data::Part::Part()
{
	resetBoundingBox();
}

void Data::Part::resetBoundingBox()
{
	Eigen::Vector3f min, max;
	min << 100.0, 100.0, 100.0;
	max << -100.0, -100.0, -100.0;
	boundingBox = Eigen::AlignedBox3f(min, max);
}

void Data::Part::recalculateBoundingBox(Data::Vertex* vertex)
{
	if (vertex->pos[0] < boundingBox.min()[0]) {
		boundingBox.min()[0] = vertex->pos[0];
	}
	if (vertex->pos[1] < boundingBox.min()[1]) {
		boundingBox.min()[1] = vertex->pos[1];
	}
	if (vertex->pos[2] < boundingBox.min()[2]) {
		boundingBox.min()[2] = vertex->pos[2];
	}
	if (vertex->pos[0] > boundingBox.max()[0]) {
		boundingBox.max()[0] = vertex->pos[0];
	}
	if (vertex->pos[1] > boundingBox.max()[1]) {
		boundingBox.max()[1] = vertex->pos[1];
	}
	if (vertex->pos[2] > boundingBox.max()[2]) {
		boundingBox.max()[2] = vertex->pos[2];
	}
}

bool Data::Part::isChild(Data::Part* parent)
{
	if (parent->type == Data::Part::Type::FOUR_LEGGED ||
	    parent->type == Data::Part::Type::SINGLE_LEGGED ||
	    parent->type == Data::Part::Type::TWO_LEGGED) {
		for (int i = Data::Part::Type::LEG_SPINDLE; i <= Data::Part::Type::LEG_BACK_LEG; i++) {
			if (type == (Data::Part::Type)(i)) {
				return true;
			}
		}
	}

	return false;
}

bool Data::Part::isParent(Data::Part* child)
{
	if (type == Data::Part::Type::FOUR_LEGGED ||
	    type == Data::Part::Type::SINGLE_LEGGED ||
	    type == Data::Part::Type::TWO_LEGGED) {
		for (int i = Data::Part::Type::LEG_SPINDLE; i <= Data::Part::Type::LEG_BACK_LEG; i++) {
			if (child->type == (Data::Part::Type)(i)) {
				return true;
			}
		}
	}

	return false;
}

void Data::Part::recalculateNormals()
{
	for (auto rit = regions.begin(); rit != regions.end(); rit++) {
		(*rit)->recalculateNormals();
	}
}

void Data::Part::findNeighborsByBoxIntersection()
{
	return parent->findPartsNeighborsByBoxIntersectionForPart(this);
}

void Data::Part::findNeighborsByVertexToFaceDistance()
{
	return parent->findPartsNeighborsByVertexToFaceDistanceForPart(this);
}

void Data::Part::scale(Eigen::Vector3f scale, Eigen::Vector3f base)
{
	resetBoundingBox();
	for (auto rit = regions.begin(); rit != regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			
			vertex->pos[0] = (vertex->pos[0] - base[0]) * scale[0] + base[0];
			vertex->pos[1] = (vertex->pos[1] - base[1]) * scale[1] + base[1];
			vertex->pos[2] = (vertex->pos[2] - base[2]) * scale[2] + base[2];
			vertex->pos[3] = 1;

			region->recalculateBoundingBox(vertex);
			recalculateBoundingBox(vertex);
		}
	}
}

void Data::Part::scale(Eigen::Vector3f scale)
{
	Eigen::Vector3f base = boundingBox.min(); // min point on the corner
	// base = (boundingBox.min() + boundingBox.max()) / 2.;
	// base[2] = boundingBox.min()[2];
	return this->scale(scale, base);
}

void Data::Part::scale(Eigen::AlignedBox3f box, Eigen::Vector3f base)
{
	Eigen::Vector3f to = (box.max() - box.min());
	Eigen::Vector3f from = (boundingBox.max() - boundingBox.min());
	Eigen::Vector3f scale;
	scale[0] = to[0] / from[0];
	scale[1] = to[1] / from[1];
	scale[2] = to[2] / from[2];
	this->scale(scale, base);
}


void Data::Part::scale(Eigen::AlignedBox3f box)
{
	Eigen::Vector3f base = boundingBox.min(); // min point on the corner
	this->scale(box, base);
}

void Data::Part::translate(Eigen::Vector3f translate)
{
	resetBoundingBox();
	for (auto rit = regions.begin(); rit != regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			
			vertex->pos[0] = (vertex->pos[0] + translate[0]);
			vertex->pos[1] = (vertex->pos[1] + translate[1]);
			vertex->pos[2] = (vertex->pos[2] + translate[2]);
			vertex->pos[3] = 1;

			region->recalculateBoundingBox(vertex);
			recalculateBoundingBox(vertex);
		}
	}
}

void Data::Part::addVertexToPartIntersection(Data::Part* part, Data::Vertex* vertex, bool mine)
{
	for (auto it = neighbors.begin(); it != neighbors.end(); it++) {
		if ((*it)->neighbor == part) {
			(*it)->vertices.push_back(vertex);
			if (mine) {
				(*it)->myVertices.push_back(vertex);
			} else {
				(*it)->neighborVertices.push_back(vertex);
			}
			(*it)->recalculateBoundingBox(vertex);
			return;
		}
	}

	Data::PartIntersection* partIntersection = new Data::PartIntersection();
	partIntersection->neighbor = part;
	partIntersection->vertices.push_back(vertex);
	if (mine) {
		partIntersection->myVertices.push_back(vertex);
	} else {
		partIntersection->neighborVertices.push_back(vertex);
	}
	partIntersection->recalculateBoundingBox(vertex);

	neighbors.push_back(partIntersection);
}
