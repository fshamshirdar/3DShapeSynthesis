#include "mix_match.h"
#include "mixers/hull_grid_points.h"
#include <set>

#include <iostream>

#define SCALE_BOUNDING_BOX false
#define K_NEAREST_NEIGHBORS 8

Data* HullGridPoints::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);

	if (SCALE_BOUNDING_BOX) {
		Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
		ref->scale(target->boundingBox, baseScale);
	}

//	chair1->findPartsNeighborsByVertexToFaceDistanceForPart(target);
//	chair2->findPartsNeighborsByVertexToFaceDistanceForPart(ref);

	


	for (auto rrit = ref->regions.begin(); rrit != ref->regions.end(); rrit++) {
		for (auto vrrit = (*rrit)->vertices.begin(); vrrit != (*rrit)->vertices.end(); vrrit++) {
			Data::Vertex* closestVertex = NULL;
			float minDist = 1000.0;
			for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
				for (auto vtnit = (*tnit)->vertices.begin(); vtnit != (*tnit)->vertices.end(); vtnit++) {
					float dist = ((*vrnit)->pos - (*vtnit)->pos).norm();
					if (dist < minDist) {
						minDist = dist;
						closestVertex = (*vtnit);
					}
				}
			}
			HullGridPoints::ClosestVertexPair* pair = new HullGridPoints::ClosestVertexPair;
			pair->translation = (closestVertex->pos - (*vrnit)->pos);
			pair->vertex = (*vrnit);
			pair->pair = closestVertex;
			pair->dist = minDist;
			controlPoints.push_back(pair);
		}
	}


	std::vector<HullGridPoints::ClosestVertexPair*> controlPoints;

	for (auto rnit = ref->neighbors.begin(); rnit != ref->neighbors.end(); rnit++) {
		for (auto vrnit = (*rnit)->vertices.begin(); vrnit != (*rnit)->vertices.end(); vrnit++) {
			Data::Vertex* closestVertex = NULL;
			float minDist = 1000.0;
			for (auto tnit = target->neighbors.begin(); tnit != target->neighbors.end(); tnit++) {
				for (auto vtnit = (*tnit)->vertices.begin(); vtnit != (*tnit)->vertices.end(); vtnit++) {
					float dist = ((*vrnit)->pos - (*vtnit)->pos).norm();
					if (dist < minDist) {
						minDist = dist;
						closestVertex = (*vtnit);
					}
				}
			}
			HullGridPoints::ClosestVertexPair* pair = new HullGridPoints::ClosestVertexPair;
			pair->translation = (closestVertex->pos - (*vrnit)->pos);
			pair->vertex = (*vrnit);
			pair->pair = closestVertex;
			pair->dist = minDist;
			controlPoints.push_back(pair);
		}
	}

	ref->resetBoundingBox();
	for (auto rit = ref->regions.begin(); rit != ref->regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			float sum = 0.;

			std::vector<HullGridPoints::VertexDist> kClosestPoints;
			kClosestPoints.reserve(controlPoints.size());

			for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
				HullGridPoints::VertexDist vd;
				vd.translation = (*cpit)->translation;
				vd.vertex = (*cpit)->vertex;
				vd.dist = (vertex->pos - (*cpit)->vertex->pos).norm();
				kClosestPoints.push_back(vd);
			}

			std::sort(kClosestPoints.begin(), kClosestPoints.end());

			Eigen::Vector4f translation = Eigen::Vector4f::Zero();

			int len = 0;
			auto it = kClosestPoints.begin();
			for (int i=0; i<K_NEAREST_NEIGHBORS && it != kClosestPoints.end(); i++, it++) {
				sum += it->dist;
				len ++;
			}

 			it = kClosestPoints.begin();
			for (int i=0; i<len && it != kClosestPoints.end(); i++, it++) { // three closest base points
				translation += ((sum - it->dist) / ((len-1)*sum)) * it->translation;
			}
			vertex->pos += translation;
			vertex->pos[3] = 1;

			ref->recalculateBoundingBox(vertex);
			region->recalculateBoundingBox(vertex);
		}
	}

	output->replacePartByType(ref);

	return output;
}

HullGridPoints::HullGridPoint** HullGridPoints::findHullPoints(Data::Part* part, int n)
{
	HullGridPoints::HullGridPoint*** points = HullGridPoints::HullGridPoint**[2]; // back and front
	points[0] = new HullGridPoints::GridPoint*[n];
	points[1] = new HullGridPoints::GridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridPoints::GridPoint[n]();
	}

	// XZ plane
	Eigen::Vector3f min = part->boundingBox.min();
	float width = part->boundingBox.max()[0] - part->boundingBox.min()[0];
	float height = part->boundingBox.max()[2] - part->boundingBox.min()[2];

	float cellWidth = width / n;
	float cellHeight = height / n;

	for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
		Data::Region* region = (*rit);
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			int i = int((vertex->pos[0] - min[0]) / cellWidth);
			int j = int((vertex->pos[2] - min[2]) / cellHeight);
			Eigen::Vector4f mid;
			mid << cellWidth;
			if (points[0][i][j]
		}
	}	
}
