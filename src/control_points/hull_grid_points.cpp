#include "mix_match.h"
#include "control_points/hull_grid_points.h"
#include <set>

#include <iostream>

HullGridPoints::HullGridPoints(int nx, int ny, int nz) : nx(nx), ny(ny), nz(nz)
{
}

std::vector<ControlPointsMiner::ControlPoint*> HullGridPoints::findControlPoints(Data::Part* ref, Data::Part* target)
{
	scaleToTarget(ref, target);

	HullGridPoints::HullGridPoint*** refPoints;
	HullGridPoints::HullGridPoint*** targetPoints;

	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;

	// XZ
	if (nx != 0 && nz != 0) {
		refPoints = findXZHullPoints(ref, nx, nz);
		targetPoints = findXZHullPoints(target, nx, nz);
		std::vector<ControlPointsMiner::ControlPoint*> controlPointsXZ = findCorrespondingPoints(refPoints, targetPoints, nx, nz);
		controlPoints.insert(controlPoints.end(), controlPointsXZ.begin(), controlPointsXZ.end());
	}
	// XY
	if (nx != 0 && ny != 0) {
		refPoints = findXYHullPoints(ref, nx, ny);
		targetPoints = findXYHullPoints(target, nx, ny);
		std::vector<ControlPointsMiner::ControlPoint*> controlPointsXY = findCorrespondingPoints(refPoints, targetPoints, nx, ny);
		controlPoints.insert(controlPoints.end(), controlPointsXY.begin(), controlPointsXY.end());
	}
	// YZ
	if (ny != 0 && nz != 0) {
		refPoints = findYZHullPoints(ref, ny, nz);
		targetPoints = findYZHullPoints(target, ny, nz);
		std::vector<ControlPointsMiner::ControlPoint*> controlPointsYZ = findCorrespondingPoints(refPoints, targetPoints, ny, nz);
		controlPoints.insert(controlPoints.end(), controlPointsYZ.begin(), controlPointsYZ.end());
	}
	unscaleToRef(ref, target);

	return controlPoints;
}

std::vector<ControlPointsMiner::ControlPoint*> HullGridPoints::findCorrespondingPoints(HullGridPoints::HullGridPoint*** refPoints, HullGridPoints::HullGridPoint*** targetPoints, int n, int m)
{
	std::vector<ControlPointsMiner::ControlPoint*> controlPoints;
	for (int i=0; i<2; i++) {
		for (int j=0; j<n; j++) {
			for (int k=0; k<m; k++) {
				if (refPoints[i][j][k].vertex && targetPoints[i][j][k].vertex &&
				    refPoints[0][j][k].vertex != refPoints[1][j][k].vertex &&
				    targetPoints[0][j][k].vertex != targetPoints[1][j][k].vertex) {
					ControlPointsMiner::ControlPoint* pair = new ControlPointsMiner::ControlPoint;
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

			frontMid << min[0] + i * cellWidth + cellWidth / 2., min[1], min[2] + j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << min[0] + i * cellWidth + cellWidth / 2., max[1], min[2] + j * cellHeight + cellHeight / 2., 1.;
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

			frontMid << min[0] + i * cellWidth + cellWidth / 2., min[1] + j * cellHeight + cellHeight / 2., min[2], 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << min[0] + i * cellWidth + cellWidth / 2., min[1] + j * cellHeight + cellHeight / 2., max[2], 1.;
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

			frontMid << min[0], min[1] + i * cellWidth + cellWidth / 2., min[2] + j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - frontMid).norm();
			if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
				points[0][i][j].vertex = vertex;
				points[0][i][j].dist = dist;
			}

			backMid << max[0], min[1] + i * cellWidth + cellWidth / 2., min[2] + j * cellHeight + cellHeight / 2., 1.;
			dist = (vertex->pos - backMid).norm();
			if (! points[1][i][j].vertex || dist < points[1][i][j].dist) {
				points[1][i][j].vertex = vertex;
				points[1][i][j].dist = dist;
			}
		}
	}

	return points;
}
