#include "mix_match.h"
#include "mixers/hull_grid_points.h"
#include <set>

#include <iostream>

#define K_NEAREST_NEIGHBORS 8
#define GRID_X 10
#define GRID_Y 10
#define GRID_Z 10
#define ENABLE_XZ true
#define ENABLE_XY true
#define ENABLE_YZ true

Data* HullGridPoints::mix(Data* chair1, Data* chair2)
{
	Data* output = chair1;
	Data::Part* target = chair1->findPartByType(Data::Part::BACK_SHEET);
	Data::Part* ref = chair2->findPartByType(Data::Part::BACK_SHEET);

	Eigen::Vector3f baseScale = (target->boundingBox.min() + target->boundingBox.max()) / 2.;
	ref->scale(target->boundingBox, baseScale);
	ref->transform(target->boundingBox.min() - ref->boundingBox.min());

	HullGridPoints::HullGridPoint*** refPoints;
	HullGridPoints::HullGridPoint*** targetPoints;
	std::vector<HullGridPoints::HullVertexPair*> controlPoints;

	// XZ
	refPoints = findXZHullPoints(ref, GRID_X, GRID_Z);
	targetPoints = findXZHullPoints(target, GRID_X, GRID_Z);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsXZ = findCorrespondingPoints(refPoints, targetPoints, GRID_X, GRID_Z);
	controlPoints.insert(controlPoints.end(), controlPointsXZ.begin(), controlPointsXZ.end());
	// XY
	refPoints = findXYHullPoints(ref, GRID_X, GRID_Y);
	targetPoints = findXYHullPoints(target, GRID_X, GRID_Y);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsXY = findCorrespondingPoints(refPoints, targetPoints, GRID_X, GRID_Y);
	controlPoints.insert(controlPoints.end(), controlPointsXY.begin(), controlPointsXY.end());
	// YZ
	refPoints = findYZHullPoints(ref, GRID_Y, GRID_Z);
	targetPoints = findYZHullPoints(target, GRID_Y, GRID_Z);
	std::vector<HullGridPoints::HullVertexPair*> controlPointsYZ = findCorrespondingPoints(refPoints, targetPoints, GRID_Y, GRID_Z);
	controlPoints.insert(controlPoints.end(), controlPointsYZ.begin(), controlPointsYZ.end());

	for (auto cpit=controlPoints.begin(); cpit != controlPoints.end(); cpit++) {
		controlPointVertices.push_back((*cpit)->vertex);
	}
	std::cout << "Controller points size: " << controlPoints.size() << std::endl;

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

std::vector<HullGridPoints::HullVertexPair*> HullGridPoints::findCorrespondingPoints(HullGridPoints::HullGridPoint*** refPoints, HullGridPoints::HullGridPoint*** targetPoints, int n, int m)
{
	std::vector<HullGridPoints::HullVertexPair*> controlPoints;
	for (int i=0; i<2; i++) {
		for (int j=0; j<n; j++) {
			for (int k=0; k<m; k++) {
				if (refPoints[i][j][k].vertex && targetPoints[i][j][k].vertex) {
					HullGridPoints::HullVertexPair* pair = new HullGridPoints::HullVertexPair;
					pair->translation = (targetPoints[i][j][k].vertex->pos - refPoints[i][j][k].vertex->pos);
					pair->vertex = refPoints[i][j][k].vertex;
					pair->pair = targetPoints[i][j][k].vertex;
					controlPoints.push_back(pair);
				}
			}
		}
	}
	return controlPoints;
}

HullGridPoints::HullGridPoint*** HullGridPoints::findXZHullPoints(Data::Part* part, int n, int m)
{
	HullGridPoints::HullGridPoint*** points = new HullGridPoints::HullGridPoint**[2]; // back and front
	points[0] = new HullGridPoints::HullGridPoint*[n];
	points[1] = new HullGridPoints::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridPoints::HullGridPoint[m]();
		points[1][i] = new HullGridPoints::HullGridPoint[m]();
	}

	// XZ plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[0] - min[0];
	float height = max[2] - min[2];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
		Data::Region* region = (*rit);
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			int i = int((vertex->pos[0] - min[0]) / cellWidth);
			int j = int((vertex->pos[2] - min[2]) / cellHeight);
			i = (i >= n) ? n-1 : i;
			j = (j >= m) ? m-1 : j;
			Eigen::Vector4f frontMid;
			Eigen::Vector4f backMid;
			float dist;

			frontMid << i * cellWidth + cellWidth / 2., min[1], j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << i * cellWidth + cellWidth / 2., max[1], j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - backMid).norm();
			if (! points[1][i][j].vertex || dist < points[1][i][j].dist) {
				points[1][i][j].vertex = vertex;
				points[1][i][j].dist = dist;
			}
		}
	}

	return points;
}

HullGridPoints::HullGridPoint*** HullGridPoints::findXYHullPoints(Data::Part* part, int n, int m)
{
	HullGridPoints::HullGridPoint*** points = new HullGridPoints::HullGridPoint**[2]; // back and front
	points[0] = new HullGridPoints::HullGridPoint*[n];
	points[1] = new HullGridPoints::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridPoints::HullGridPoint[m]();
		points[1][i] = new HullGridPoints::HullGridPoint[m]();
	}

	// XY plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[0] - min[0];
	float height = max[1] - min[1];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
		Data::Region* region = (*rit);
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			int i = int((vertex->pos[0] - min[0]) / cellWidth);
			int j = int((vertex->pos[1] - min[1]) / cellHeight);
			i = (i >= n) ? n-1 : i;
			j = (j >= m) ? m-1 : j;
			Eigen::Vector4f frontMid;
			Eigen::Vector4f backMid;
			float dist;

			frontMid << i * cellWidth + cellWidth / 2., j * cellHeight + cellHeight / 2., min[2], 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << i * cellWidth + cellWidth / 2., j * cellHeight + cellHeight / 2., max[2], 1.;
			dist = (vertex->pos - backMid).norm();
			if (! points[1][i][j].vertex || dist < points[1][i][j].dist) {
				points[1][i][j].vertex = vertex;
				points[1][i][j].dist = dist;
			}
		}
	}

	return points;
}

HullGridPoints::HullGridPoint*** HullGridPoints::findYZHullPoints(Data::Part* part, int n, int m)
{
	HullGridPoints::HullGridPoint*** points = new HullGridPoints::HullGridPoint**[2]; // back and front
	points[0] = new HullGridPoints::HullGridPoint*[n];
	points[1] = new HullGridPoints::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridPoints::HullGridPoint[m]();
		points[1][i] = new HullGridPoints::HullGridPoint[m]();
	}

	// YZ plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[1] - min[1];
	float height = max[2] - min[2];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
		Data::Region* region = (*rit);
		for (auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++) {
			Data::Vertex* vertex = (*vit);
			int i = int((vertex->pos[1] - min[1]) / cellWidth);
			int j = int((vertex->pos[2] - min[2]) / cellHeight);
			i = (i >= n) ? n-1 : i;
			j = (j >= m) ? m-1 : j;
			Eigen::Vector4f frontMid;
			Eigen::Vector4f backMid;
			float dist;

			frontMid << min[0], i * cellWidth + cellWidth / 2., j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << max[0], i * cellWidth + cellWidth / 2., j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - backMid).norm();
			if (! points[1][i][j].vertex || dist < points[1][i][j].dist) {
				points[1][i][j].vertex = vertex;
				points[1][i][j].dist = dist;
			}
		}
	}

	return points;
}
