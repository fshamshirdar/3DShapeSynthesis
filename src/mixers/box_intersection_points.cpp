#include "mix_match.h"
#include "mixers/box_intersection_points.h"
#include "mixers/hull_grid_points.h"
#include <set>

#include <iostream>

#define K_NEAREST_NEIGHBORS 8
#define GRID_X 20
#define GRID_Y 20
#define GRID_Z 20

Data* BoxIntersectionPoints::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);

	// Find intersection Points
//	chair1->findPartsNeighborsByVertexToFaceDistanceForPart(target);
//	chair2->findPartsNeighborsByVertexToFaceDistanceForPart(ref);
	chair1->findPartsNeighborsByBoxIntersectionForPart(target);
	chair2->findPartsNeighborsByBoxIntersectionForPart(ref);

	// Hull Grid Points
	Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	ref->transform(target->boundingBox.min() - ref->boundingBox.min());

	HullGridPoints* hullGridMixer = new HullGridPoints;
	HullGridPoints::HullGridPoint*** refPoints;
	HullGridPoints::HullGridPoint*** targetPoints;

	// TODO: merge them all
	std::vector<HullGridPoints::HullVertexPair*> intersectionPoints;

	// XZ
	refPoints = hullGridMixer->findXZHullPoints(ref, GRID_X, GRID_Z);
	targetPoints = hullGridMixer->findXZHullPoints(target, GRID_X, GRID_Z);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsXZ = hullGridMixer->findCorrespondingPoints(refPoints, targetPoints, GRID_X, GRID_Z);
	intersectionPoints.insert(intersectionPoints.end(), controlPointsXZ.begin(), controlPointsXZ.end());
	// XY
	refPoints = hullGridMixer->findXYHullPoints(ref, GRID_X, GRID_Y);
	targetPoints = hullGridMixer->findXYHullPoints(target, GRID_X, GRID_Y);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsXY = hullGridMixer->findCorrespondingPoints(refPoints, targetPoints, GRID_X, GRID_Y);
	intersectionPoints.insert(intersectionPoints.end(), controlPointsXY.begin(), controlPointsXY.end());
	// YZ
	refPoints = hullGridMixer->findYZHullPoints(ref, GRID_Y, GRID_Z);
	targetPoints = hullGridMixer->findYZHullPoints(target, GRID_Y, GRID_Z);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsYZ = hullGridMixer->findCorrespondingPoints(refPoints, targetPoints, GRID_Y, GRID_Z);
	intersectionPoints.insert(intersectionPoints.end(), controlPointsYZ.begin(), controlPointsYZ.end());

	std::vector<BoxIntersectionPoints::VertexPair*> controlPoints;
	for (auto it=intersectionPoints.begin(); it != intersectionPoints.end(); it++) {
		BoxIntersectionPoints::VertexPair* pair = new BoxIntersectionPoints::VertexPair;
		pair->translation = (*it)->translation;
		pair->vertex = (*it)->vertex;
		pair->pair = (*it)->pair;
		pair->dist = 0.; // not using
		// controlPoints.push_back(pair);
	}
	// TODO till here

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
			BoxIntersectionPoints::VertexPair* pair = new BoxIntersectionPoints::VertexPair;
			pair->translation = (closestVertex->pos - (*vrnit)->pos);
			pair->vertex = (*vrnit);
			pair->pair = closestVertex;
			pair->dist = minDist;
			controlPoints.push_back(pair);
		}
	}

	for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
		controlPointVertices.push_back((*cpit)->vertex);
	}

	ref->resetBoundingBox();
	for (auto rit = ref->regions.begin(); rit != ref->regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			float sum = 0.;

			std::vector<BoxIntersectionPoints::VertexDist> kClosestPoints;
			kClosestPoints.reserve(controlPoints.size());

			for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
				BoxIntersectionPoints::VertexDist vd;
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
