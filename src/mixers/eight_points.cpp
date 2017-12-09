#include "mix_match.h"
#include "mixers/eight_points.h"
#include <set>

#include <iostream>

#define SCALE_BOUNDING_BOX true
#define K_NEAREST_NEIGHBORS 8

Data* EightPoints::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::SEAT_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::SEAT_SHEET);

	if (SCALE_BOUNDING_BOX) {
		scaleBoundingBox(ref, target);
	}

	Data::Vertex** refPoints = find8Points(ref);
	Data::Vertex** targetPoints = find8Points(target);

	Eigen::Vector4f translations[8];
	for (int i=0; i<8; i++) {
		translations[i] = targetPoints[i]->pos - refPoints[i]->pos;
	}

	float dists[8], sum;

	ref->resetBoundingBox();
	for (auto rit = ref->regions.begin(); rit != ref->regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			sum = 0.;

			std::set<VertexDist> icp;

			for (int i=0; i<8; i++) {
				dists[i] = (vertex->pos - targetPoints[i]->pos).norm();
				sum += dists[i];

				EightPoints::VertexDist vd;
				vd.translation = translations[i];
				vd.vertex = vertex;
				vd.dist = dists[i];
				icp.insert(vd);
			}

			Eigen::Vector4f translation = Eigen::Vector4f::Zero();

			auto it = icp.begin();
			for (int i=0; i<K_NEAREST_NEIGHBORS; i++, it++) {
				sum += it->dist;
			}

 			it = icp.begin();
			for (int i=0; i<K_NEAREST_NEIGHBORS; i++, it++) { // three closest base points
				translation += ((sum-it->dist) / ((K_NEAREST_NEIGHBORS-1)*sum)) * it->translation;
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

Data::Vertex** EightPoints::find8Points(Data::Part* part)
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
