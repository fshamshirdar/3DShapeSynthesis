#include "mix_match.h"
#include "mixers/knn_weighted_interpolation.h"
#include <set>

#include <iostream>

KNNWeightedInterpolation::KNNWeightedInterpolation(std::vector<ControlPointsMiner*> miners, int K, float maxDist) : miners(miners), K(K), maxDist(maxDist)
{
	this->name = "knn";
}

Data* KNNWeightedInterpolation::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	//Data::Part* refBack = chair2->findPartByType(Data::Part::SEAT_SHEET);
	//Data::Part* targetBack = chair1->findPartByType(Data::Part::SEAT_SHEET);
	Data::Part* refBack = chair2->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* targetBack = chair1->findPartByType(Data::Part::BACK_SHEET);
//	Data::Part* refBack = chair2->findPartByType(Data::Part::LEG);
//	Data::Part* targetBack = chair1->findPartByType(Data::Part::LEG);
	refBack = mixPart(refBack, targetBack);
	output->replacePartByType(refBack);

	return output;
}

Data::Part* KNNWeightedInterpolation::mixPart(Data::Part* ref, Data::Part* target)
{
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;
	std::pair<Eigen::Matrix3f, Eigen::Vector3f> transformation;
	std::vector<Eigen::Vector3f> sourcePoints, targetPoints;
	for (auto mit = miners.begin(); mit != miners.end(); mit++)
	{
		std::vector<ControlPointsMiner::ControlPoint*> minerControlPoints;
		minerControlPoints = (*mit)->findControlPoints(ref, target);
		for (auto cit = minerControlPoints.begin(); cit != minerControlPoints.end(); cit++) {
			sourcePoints.push_back((*cit)->vertex->pos.head<3>()); 
			targetPoints.push_back((*cit)->pair->pos.head<3>()); 
		}

		controlPoints.insert(controlPoints.end(), minerControlPoints.begin(), minerControlPoints.end());
		totalControlPoints.insert(totalControlPoints.end(), minerControlPoints.begin(), minerControlPoints.end());
	}

/*
	// Rigid transform first
	if (sourcePoints.size() > 3) {
		transformation = Utils::computeRigidTransform(sourcePoints, targetPoints);
		ref->transform(transformation);
	}
	ref->recalculateNormals();
*/


	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (target->boundingBox.min() - ref->boundingBox.min());
	ref->translate(translation);

	// TODO: do we really need this?
	for (auto cit = controlPoints.begin(); cit != controlPoints.end(); cit++) {
		(*cit)->translation = ((*cit)->pair->pos - (*cit)->vertex->pos);
	}

	ref->resetBoundingBox();
	for (auto rit = ref->regions.begin(); rit != ref->regions.end(); rit++) {
		Data::Region* region = (*rit);
		region->resetBoundingBox();
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			float sum = 0.;

			std::vector<ControlPointsMiner::VertexControlPoint> kClosestPoints;
			kClosestPoints.reserve(controlPoints.size());

			for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
				ControlPointsMiner::VertexControlPoint vd;
				vd.translation = (*cpit)->translation;
				vd.vertex = (*cpit)->vertex;
				vd.dist = (vertex->pos - (*cpit)->vertex->pos).norm();
				if (vd.dist < maxDist) {
					kClosestPoints.push_back(vd);
				}
			}

			std::sort(kClosestPoints.begin(), kClosestPoints.end());

			Eigen::Vector4f translation = Eigen::Vector4f::Zero();

			int len = 0;
			auto it = kClosestPoints.begin();
			for (int i=0; i<K && it != kClosestPoints.end(); i++, it++) {
				sum += it->dist;
				len ++;
			}

 			it = kClosestPoints.begin();
			for (int i=0; i<len && it != kClosestPoints.end(); i++, it++) {
				translation += ((sum - it->dist) / ((len-1)*sum)) * it->translation;
			}
			vertex->pos += translation;
			vertex->pos[3] = 1;

			ref->recalculateBoundingBox(vertex);
			region->recalculateBoundingBox(vertex);
		}
	}
	ref->recalculateNormals();

	return ref;
}
