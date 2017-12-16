#include "mix_match.h"
#include <iostream>
#include "control_points/eight_points.h"
#include "control_points/closest_connecting_points.h"
#include "control_points/hull_grid_points.h"
#include "control_points/hull_grid_rays.h"
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

	// Data::Part::Type type = static_cast<Data::Part::Type>((rand() % Data::Part::Type::LEG) + 1);
	// std::cout << "selected type: " << type << std::endl;
	int category = rand() % 9;
	std::cout << "selected category: " << category << std::endl;

	switch(category) {
		case 0:
			data1 = mixBack(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 1:
			data1 = mixHandles(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 2:
			data1 = mixLeg(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 3:
			data1 = mixFrontLeg(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 4:
			data1 = mixBackLeg(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 5:
			data1 = mixBar(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 6:
			data1 = mixBase(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 7:
			data1 = mixBranch(data1, data2);
			if (! data1) {
				return mix();
			}
		break;
		case 8:
		case Data::Part::LEG_LEFT_SPINDLE:
		default:
			return mix();
	}
	
	int index = rand() % 10000;
	std::stringstream ss;
	// ss << "NEW_" << data1->path << "_" << data2->path << "_" << index;
	ss << "NEW_" << "_" << index;
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
	if (rand() % 2 == 0) {
		controlPointsMiners.push_back(new EightPoints());
	}
	controlPointsMiners.push_back(new BoxIntersectionPoints());
//	controlPointsMiners.push_back(new MissingConnectingPoints());
	if (rand() % 2 == 0) {
		controlPointsMiners.push_back(new HullGridPoints(grid_x, 0, grid_z));
	} else {
		controlPointsMiners.push_back(new HullGridRays(grid_x, 0, grid_z));
	}

	// Mixers
	int K = rand() % 10 + 6;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	std::vector<MixMatch*> mixers;
	mixers.push_back(new RigidTransform(controlPointsMiners));
//	mixers.push_back(new SimpleBox());
	mixers.push_back(new IntersectionBox());
	mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	std::cout << "mixer: " << mixer->name << std::endl;

	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixHandles(Data* data1, Data* data2)
{
	Data* data1Clone = data1->clone();
	Data* data2Clone = data2->clone();

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
		if (rand() % 2 == 0) {
			controlPointsMiners.push_back(new EightPoints());
		}
		controlPointsMiners.push_back(new MissingConnectingPoints());
		controlPointsMiners.push_back(new BoxIntersectionPoints());

		// Mixers
		int K = rand() % 10 + 7;
		float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
		std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

		mixers.push_back(new RigidTransform(controlPointsMiners));
//		mixers.push_back(new SimpleBox());
		mixers.push_back(new IntersectionBox());
		mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
	}

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());
	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}

	// Right handles
	Data::Part* target2 = data1Clone->findPartByType(Data::Part::Type::RIGHT_HANDLE);
	Data::Part* ref2 = data2Clone->findPartByType(Data::Part::Type::RIGHT_HANDLE);
	if (missing) {
		target2 = data1Clone->findPartByType(Data::Part::Type::SEAT_SHEET);
	}

	ref2 = mixer->mixPart(ref2, target2);
	if (! ref2) {
		return NULL;
	}

	data1->replacePartByType(ref);
	data1->replacePartByType(ref2);

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

	int K = rand() % 10 + 6;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	std::vector<MixMatch*> mixers;
	if (ref->type == Data::Part::Type::FOUR_LEGGED) {
		if (target->type == Data::Part::Type::FOUR_LEGGED) {
			std::vector<ControlPointsMiner*> controlPointsMiners;

			if (rand() % 2 == 0) {
			controlPointsMiners.push_back(new EightPoints());
			}
			controlPointsMiners.push_back(new BoxIntersectionPoints());
			if (rand() % 5 == 0) {
				controlPointsMiners.push_back(new MissingConnectingPoints());
			}
			if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridPoints(grid_x, 0, grid_z));
			} else if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridRays(grid_x, 0, grid_z));
			}


			mixers.push_back(new RigidTransform(controlPointsMiners));
//			mixers.push_back(new MissingIntersectionPart());
//			mixers.push_back(new MissingPart());
//			mixers.push_back(new SimpleBox());
			mixers.push_back(new IntersectionBox());
			mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
		} else { // different style
			std::vector<ControlPointsMiner*> controlPointsMiners;
			controlPointsMiners.push_back(new MissingConnectingPoints());

			mixers.push_back(new RigidTransform(controlPointsMiners));
			mixers.push_back(new MissingIntersectionPart());
			mixers.push_back(new MissingPart());
//			mixers.push_back(new SimpleBox());
		}
	} else if (ref->type == Data::Part::Type::SINGLE_LEGGED) {
		if (target->type == Data::Part::Type::SINGLE_LEGGED) {
			std::vector<ControlPointsMiner*> controlPointsMiners;
			if (rand() % 2 == 0) {
				controlPointsMiners.push_back(new EightPoints());
			}
			controlPointsMiners.push_back(new BoxIntersectionPoints());
			if (rand() % 5 == 0) {
				controlPointsMiners.push_back(new MissingConnectingPoints());
			}
			if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridPoints(grid_x, grid_y, grid_z));
			} else if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridRays(grid_x, grid_y, grid_z));
			}

			mixers.push_back(new RigidTransform(controlPointsMiners));
//			mixers.push_back(new MissingIntersectionPart());
//			mixers.push_back(new MissingPart());
//			mixers.push_back(new SimpleBox());
			mixers.push_back(new IntersectionBox());
			mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
		} else { // different style
			std::vector<ControlPointsMiner*> controlPointsMiners;
			controlPointsMiners.push_back(new MissingConnectingPoints());

			mixers.push_back(new RigidTransform(controlPointsMiners));
			mixers.push_back(new MissingIntersectionPart());
			mixers.push_back(new MissingPart());
//			mixers.push_back(new SimpleBox());
//			mixers.push_back(new IntersectionBox());
		}
	} else if (ref->type == Data::Part::Type::TWO_LEGGED) {
		if (target->type == Data::Part::Type::TWO_LEGGED) {
			std::vector<ControlPointsMiner*> controlPointsMiners;
			if (rand() % 2 == 0) {
				controlPointsMiners.push_back(new EightPoints());
			}
			controlPointsMiners.push_back(new BoxIntersectionPoints());
			if (rand() % 5 == 0) {
				controlPointsMiners.push_back(new MissingConnectingPoints());
			}
			if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridPoints(grid_x, grid_y, grid_z));
			} else if (rand() % 3 == 0) {
				controlPointsMiners.push_back(new HullGridRays(grid_x, grid_y, grid_z));
			}

			mixers.push_back(new RigidTransform(controlPointsMiners));
//			mixers.push_back(new MissingIntersectionPart());
//			mixers.push_back(new MissingPart());
//			mixers.push_back(new SimpleBox());
			mixers.push_back(new IntersectionBox());
			mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
		} else { // different style - NO! return NULL
			return NULL;
		}
	}

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixFrontLeg(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG_FRONT_LEG);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG_FRONT_LEG);

	int grid_x = rand() % 5 + 2;
	int grid_y = rand() % 5 + 2;
	int grid_z = rand() % 5 + 2;

	int K = rand() % 10 + 6;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	if (target->regions.size() == 0 || ref->regions.size() == 0) {
		return NULL;
	}

	std::vector<MixMatch*> mixers;
	std::vector<ControlPointsMiner*> controlPointsMiners;

	if (rand() % 2 == 0) {
		controlPointsMiners.push_back(new EightPoints());
	}
	controlPointsMiners.push_back(new BoxIntersectionPoints());
	if (rand() % 5 == 0) {
		controlPointsMiners.push_back(new MissingConnectingPoints());
	}
	if (rand() % 3 == 0) {
		controlPointsMiners.push_back(new HullGridPoints(grid_x, grid_y, grid_z));
	} else if (rand() % 3 == 0) {
		controlPointsMiners.push_back(new HullGridRays(grid_x, grid_y, grid_z));
	}

	mixers.push_back(new RigidTransform(controlPointsMiners));
//	mixers.push_back(new MissingIntersectionPart());
//	mixers.push_back(new MissingPart());
//	mixers.push_back(new SimpleBox());
	mixers.push_back(new IntersectionBox());
	mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixBackLeg(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG_BACK_LEG);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG_BACK_LEG);

	int grid_x = rand() % 5 + 2;
	int grid_y = rand() % 5 + 2;
	int grid_z = rand() % 5 + 2;

	int K = rand() % 10 + 6;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	if (target->regions.size() == 0 || ref->regions.size() == 0) {
		return NULL;
	}

	std::vector<MixMatch*> mixers;
	std::vector<ControlPointsMiner*> controlPointsMiners;
	if (rand() % 2 == 0) {
		controlPointsMiners.push_back(new EightPoints());
	}
	controlPointsMiners.push_back(new BoxIntersectionPoints());
	if (rand() % 5 == 0) {
		controlPointsMiners.push_back(new MissingConnectingPoints());
	}
	if (rand() % 3 == 0) {
		controlPointsMiners.push_back(new HullGridPoints(grid_x, grid_y, grid_z));
	} else if (rand() % 3 == 0) {
		controlPointsMiners.push_back(new HullGridRays(grid_x, grid_y, grid_z));
	}

	mixers.push_back(new RigidTransform(controlPointsMiners));
//	mixers.push_back(new MissingIntersectionPart());
//	mixers.push_back(new MissingPart());
//	mixers.push_back(new SimpleBox());
	mixers.push_back(new IntersectionBox());
	mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());

	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixBar(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG_BAR);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG_BAR);

	if (ref->regions.size() == 0 || target->regions.size() == 0) {
		return NULL;
	}

	std::vector<MixMatch*> mixers;
	// Control Points
	int grid_x = rand() % 20 + 5;
	int grid_z = rand() % 20 + 5;
	std::vector<ControlPointsMiner*> controlPointsMiners;
	if (rand() % 2 == 0) {
		controlPointsMiners.push_back(new EightPoints());
	}
//	controlPointsMiners.push_back(new MissingConnectingPoints());
	controlPointsMiners.push_back(new BoxIntersectionPoints());

	// Mixers
	int K = rand() % 10 + 6;
	float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
	std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

	mixers.push_back(new RigidTransform(controlPointsMiners));
//	mixers.push_back(new SimpleBox());
	mixers.push_back(new IntersectionBox());
	mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());
	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixBranch(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG_BRANCH);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG_BRANCH);

	bool missing = false;
	if (target->regions.size() == 0) {
		target = data1->findPartByType(Data::Part::Type::LEG_BASE);
		if (target->regions.size() == 0) {
			return NULL;
		}
		ref->type = Data::Part::Type::LEG_BASE;
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
		if (rand() % 2 == 0) {
			controlPointsMiners.push_back(new EightPoints());
		}
//		controlPointsMiners.push_back(new MissingConnectingPoints());
		controlPointsMiners.push_back(new BoxIntersectionPoints());

		// Mixers
		int K = rand() % 10 + 6;
		float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.1;
		std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

		mixers.push_back(new RigidTransform(controlPointsMiners));
//		mixers.push_back(new SimpleBox());
		mixers.push_back(new IntersectionBox());
		mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
	}

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());
	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}

Data* MixMatch::mixBase(Data* data1, Data* data2)
{
	Data::Part* target = data1->findPartByType(Data::Part::Type::LEG_BASE);
	Data::Part* ref = data2->findPartByType(Data::Part::Type::LEG_BASE);

	bool missing = false;
	if (target->regions.size() == 0) {
		target = data1->findPartByType(Data::Part::Type::LEG_BRANCH);
		if (target->regions.size() == 0) {
			return NULL;
		}
		ref->type = Data::Part::Type::LEG_BRANCH;
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
		if (rand() % 2 == 0) {
			controlPointsMiners.push_back(new EightPoints());
		}
//		controlPointsMiners.push_back(new MissingConnectingPoints());
		controlPointsMiners.push_back(new BoxIntersectionPoints());

		// Mixers
		int K = rand() % 10 + 6;
		float maxDist = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/2.5)) + 0.2;
		std::cout << "K: " << K << " maxDist: " << maxDist << std::endl;

		mixers.push_back(new RigidTransform(controlPointsMiners));
		mixers.push_back(new SimpleBox());
		mixers.push_back(new IntersectionBox());
		mixers.push_back(new KNNWeightedInterpolation(controlPointsMiners, K, maxDist));
	}

	std::random_shuffle(mixers.begin(), mixers.end());
	MixMatch* mixer = *(mixers.begin());
	ref = mixer->mixPart(ref, target);
	if (! ref) {
		return NULL;
	}
	data1->replacePartByType(ref);

	std::cout << "mixer: " << mixer->name << std::endl;

	totalControlPoints = mixer->totalControlPoints;

	return data1;
}
