#include "mix_match.h"
#include "mixers/rigid_transform.h"
#include <set>

#include <iostream>

RigidTransform::RigidTransform(std::vector<ControlPointsMiner*> miners) : miners(miners)
{
	this->name = "rigid";
}

Data* RigidTransform::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;

	Data::Part* refBack = chair2->findPartByType(Data::Part::SEAT_SHEET);
	Data::Part* targetBack = chair1->findPartByType(Data::Part::SEAT_SHEET);
	//Data::Part* refBack = chair2->findPartByType(Data::Part::BACK_SHEET);
	//Data::Part* targetBack = chair1->findPartByType(Data::Part::BACK_SHEET);
	//Data::Part* refBack = chair2->findPartByType(Data::Part::LEFT_HANDLE);
	//Data::Part* targetBack = chair1->findPartByType(Data::Part::SEAT_SHEET);
	refBack = mixPart(refBack, targetBack);
	output->replacePartByType(refBack);

	return output;
}

Data::Part* RigidTransform::mixPart(Data::Part* ref, Data::Part* target)
{
	Eigen::Vector3f baseScale = (ref->boundingBox.min() + ref->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	// Eigen::Vector3f translation = ((target->boundingBox.max() - ref->boundingBox.max()) + (target->boundingBox.max() - ref->boundingBox.max())) / 2.;
	Eigen::Vector3f translation = (target->boundingBox.min() - ref->boundingBox.min());
	ref->translate(translation);

	int len = 0;
	std::pair<Eigen::Matrix3f, Eigen::Vector3f> transformation;
	std::vector<Eigen::Vector3f> sourcePoints, targetPoints;
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;
	for (auto mit = miners.begin(); mit != miners.end(); mit++)
	{
		std::vector<ControlPointsMiner::ControlPoint*> minerControlPoints;
		minerControlPoints = (*mit)->findControlPoints(ref, target);
		for (auto cit = minerControlPoints.begin(); cit != minerControlPoints.end(); cit++) {
			sourcePoints.push_back((*cit)->vertex->pos.head<3>()); 
			targetPoints.push_back((*cit)->pair->pos.head<3>()); 
			len ++;
			if (len > 200) {
				break;
			}
		}
		controlPoints.insert(controlPoints.end(), minerControlPoints.begin(), minerControlPoints.end());
		totalControlPoints.insert(totalControlPoints.end(), minerControlPoints.begin(), minerControlPoints.end());
	}

	std::cout << "len: " << sourcePoints.size() << std::endl;

	if (sourcePoints.size() > 5) {
		transformation = Utils::computeRigidTransform(sourcePoints, targetPoints);
		ref->transform(transformation);
	} else {
		return NULL;
	}
	ref->recalculateNormals();

	return ref;
}
