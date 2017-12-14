#include "mix_match.h"
#include <iostream>
#include "control_points/eight_points.h"
#include "control_points/closest_connecting_points.h"
#include "control_points/hull_grid_points.h"
#include "control_points/box_intersection_points.h"
#include "control_points/box_intersection_target_points.h"
#include "control_points/missing_connecting_points.h"
#include "mixers/knn_weighted_interpolation.h"
#include "mixers/simple_box.h"
#include "mixers/intersection_box.h"
#include "mixers/missing_part.h"
#include "mixers/missing_intersection_part.h"
#include "mixers/rigid_transform.h"

void MixMatch::scaleBoundingBox(Data::Part* from, Data::Part* to)
{
	from->scale(to->boundingBox);
	from->translate(to->boundingBox.min() - from->boundingBox.min());
}

Data* MixMatch::mix()
{
	if (datas.size() < 3) {
		return NULL;
	}

	std::random_shuffle(datas.begin(), datas.end());
	Data* dataOrg1 = *(datas.begin());
	Data* dataOrg2 = *(datas.begin()+1);

	Data* data1 = dataOrg1->clone();
	Data* data2 = dataOrg2->clone();

	std::cout << data1->path << " " << data2->path << std::endl;

	Data::Part::Type type = static_cast<Data::Part::Type>((rand() % Data::Part::Type::LEG) + 1);
	std::cout << "selected type: " << type << std::endl;
	type = Data::Part::Type::LEG;

	switch(type) {
		case Data::Part::Type::BACK_SHEET:
			data1 = mixBack(data1, data2);
		break;
		case Data::Part::Type::LEFT_HANDLE:
		case Data::Part::Type::RIGHT_HANDLE:
			data1 = mixHandles(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case Data::Part::LEG:
			data1 = mixLeg(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		default:
			return mix();
	}
	
	int index = rand() % 1000;
	std::stringstream ss;
	ss << data1->path << "_" << data2->path << "_" << index;
	data1->path = ss.str();

	return data1;
}

Data* MixMatch::mixBack(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::BACK_SHEET);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::BACK_SHEET);

	// Control Points
	int grid_x = rand() % 20 + 5;
	int grid_z = rand() % 20 + 5;
	std::vector<ControlPointsMiner*> controlPointsMiners;
	controlPointsMiners.push_back(new HullGridPoints(grid_x, 0, grid_z));
	controlPointsMiners.push_back(new BoxIntersectionPoints());

	// Mixers
	int K = rand() % 10 + 3;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.)) + 0.2;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	std::vector<MixMatch*> mixers;
	mixers.push_back(new RigidTransform(controlPointsMiners));
	mixers.push_back(new SimpleBox());
	mixers.push_back(new IntersectionBox());
	mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	std::cout << "mixer: " << mixer->name << std::endl;

	ref = mixer->mixPart(ref, target);
	data1->replacePartByType(ref);

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixHandles(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEFT_HANDLE);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEFT_HANDLE);

	bool missing = false;
	if (target->regions.size() == 0) {
		target = data1->findPartByType(Data::Part::Type::SEAT_SHEET);
		missing = true;
	}
	if (ref->regions.size() == 0) {
		return NULL;
	}

	std::vector<MixMatch*> mixers;
	if (missing) {
		std::vector<ControlPointsMiner*> controlPointsMiners;
		controlPointsMiners.push_back(new MissingConnectingPoints());

		mixers.push_back(new RigidTransform(controlPointsMiners));
		mixers.push_back(new MissingIntersectionPart());
		mixers.push_back(new MissingPart());
	} else {
		// Control Points
		int grid_x = rand() % 20 + 5;
		int grid_z = rand() % 20 + 5;
		std::vector<ControlPointsMiner*> controlPointsMiners;
		controlPointsMiners.push_back(new MissingConnectingPoints());
		controlPointsMiners.push_back(new BoxIntersectionPoints());

		// Mixers
		int K = rand() % 10 + 3;
		float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.)) + 0.2;
		std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

		mixers.push_back(new RigidTransform(controlPointsMiners));
		mixers.push_back(new SimpleBox());
		mixers.push_back(new IntersectionBox());
		mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
	}

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());
	ref = mixer->mixPart(ref, target);
	data1->replacePartByType(ref);

	if (! missing) {
		target = data1->findPartByType(Data::Part::Type::RIGHT_HANDLE);
	}
	ref = data2->findPartByType(Data::Part::Type::RIGHT_HANDLE);
	ref = mixer->mixPart(ref, target);
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixLeg(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG);

	int grid_x = rand() % 5 + 2;
	int grid_y = rand() % 5 + 2;
	int grid_z = rand() % 5 + 2;

	int K = rand() % 10 + 3;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/1.)) + 0.2;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	std::vector<MixMatch*> mixers;
	if (ref->type == Data::Part::Type::FOUR_LEGGED &&
	    target->type == Data::Part::Type::FOUR_LEGGED) {
		std::vector<ControlPointsMiner*> controlPointsMiners;
		if (rand() % 5 == 0) {
			controlPointsMiners.push_back(new MissingConnectingPoints());
		}
		if (rand() % 2 == 0) {
			controlPointsMiners.push_back(new HullGridPoints(grid_x, grid_y, grid_z));
		}
		controlPointsMiners.push_back(new BoxIntersectionPoints());

		mixers.push_back(new RigidTransform(controlPointsMiners));
		mixers.push_back(new MissingIntersectionPart());
		mixers.push_back(new MissingPart());
		mixers.push_back(new SimpleBox());
		mixers.push_back(new IntersectionBox());
		mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
	} else {
		return NULL;
	}

//	FOUR_LEGGED;
//	SINGLE_LEGGED;
//	TWO_LEGGED;

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	ref = mixer->mixPart(ref, target);
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}
