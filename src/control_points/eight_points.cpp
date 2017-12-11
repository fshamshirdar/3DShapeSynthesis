#include "mix_match.h"
#include "control_points/eight_points.h"
#include <set>

#include <iostream>

std::vector<ControlPointsMiner::ControlPoint*> EightPoints::findControlPoints(Data::Part* ref, Data::Part* target)
{
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

	Data::Vertex** refVertices = find8Points(ref);
	Data::Vertex** targetVertices = find8Points(target);

	for (int i=0; i<8; i++) {
		ControlPointsMiner::ControlPoint* pair = new ControlPointsMiner::ControlPoint;
		pair->translation = (targetVertices[i]->pos - refVertices[i]->pos);
		pair->vertex = refVertices[i];
		pair->pair = targetVertices[i];
		pair->dist = 0.;
		controlPoints.push_back(pair);
	}

	return controlPoints;
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
