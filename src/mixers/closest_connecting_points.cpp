#include "mix_match.h"
#include "mixers/closest_connecting_points.h"
#include <set>

#include <iostream>

#define SCALE_BOUNDING_BOX false
#define K_NEAREST_NEIGHBORS 8

Data* ClosestConnectingPoints::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::SEAT_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::SEAT_SHEET);

	if (SCALE_BOUNDING_BOX) {
		Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
		ref->scale(target->boundingBox, baseScale);
	}

	chair1->findPartsNeighborsByVertexToFaceDistanceForPart(target);
	chair2->findPartsNeighborsByVertexToFaceDistanceForPart(ref);

	std::vector<ClosestConnectingPoints::ClosestVertexPair*> controlPoints;

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
			ClosestConnectingPoints::ClosestVertexPair* pair = new ClosestConnectingPoints::ClosestVertexPair;
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

			std::vector<ClosestConnectingPoints::VertexDist> kClosestPoints;
			kClosestPoints.reserve(controlPoints.size());

			for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
				ClosestConnectingPoints::VertexDist vd;
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

Data::Vertex** ClosestConnectingPoints::find8Points(Data::Part* part)
{
	Eigen::AlignedBox3f boundingBox = part->boundingBox;

	Eigen::Vector3f corners[8];
	corners[0] = boundingBox.corner(Eigen::AlignedBox3f::BottomLeftFloor);
	corners[1] = boundingBox.corner(Eigen::AlignedBox3f::BottomRightFloor);
	corners[2] = boundingBox.corner(Eigen::AlignedBox3f::TopLeftFloor);
	corners[3] = boundingBox.corner(Eigen::AlignedBox3f::TopRightFloor);
	corners[4] = boundingBox.corner(Eigen::AlignedBox3f::BottomLeftCeil);
	corners[5] = boundingBox.corner(Eigen::AlignedBox3f::BottomRightCeil);
	corners[6] = boundingBox.corner(Eigen::AlignedBox3f::TopLeftCeil); 
	corners[7] = boundingBox.corner(Eigen::AlignedBox3f::TopRightCeil); 

	float dists[8] = { 1000., 1000., 1000., 1000., 1000., 1000., 1000., 1000. };
	Data::Vertex** points = new Data::Vertex*[8];
	for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
		Data::Region* region = (*rit);
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);

			for (int i=0; i<8; i++) {
				float dist = (vertex->pos.head<3>() - corners[i]).norm();
				if (dist < dists[i]) {
					dists[i] = dist;
					points[i] = vertex;
				}
			}
		}
	}

	return points;
}
