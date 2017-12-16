#include "mix_match.h"
#include "control_points/hull_grid_rays.h"
#include <set>

#include <iostream>

HullGridRays::HullGridRays(int nx, int ny, int nz) : nx(nx), ny(ny), nz(nz)
{
}

std::vector<ControlPointsMiner::ControlPoint*> HullGridRays::findControlPoints(Data::Part* ref, Data::Part* target)
{
	scaleToTarget(ref, target);

	HullGridRays::HullGridPoint*** refPoints;
	HullGridRays::HullGridPoint*** targetPoints;

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

std::vector<ControlPointsMiner::ControlPoint*> HullGridRays::findCorrespondingPoints(HullGridRays::HullGridPoint*** refPoints, HullGridRays::HullGridPoint*** targetPoints, int n, int m)
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

HullGridRays::HullGridPoint*** HullGridRays::findXZHullPoints(Data::Part* part, int n, int m)
{
	HullGridRays::HullGridPoint*** points = new HullGridRays::HullGridPoint**[2]; // back and front
	points[0] = new HullGridRays::HullGridPoint*[n];
	points[1] = new HullGridRays::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridRays::HullGridPoint[m]();
		points[1][i] = new HullGridRays::HullGridPoint[m]();
	}

	// XZ plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[0] - min[0];
	float height = max[2] - min[2];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			points[0][i][j].dist = 1000.;
			points[1][i][j].dist = 0.;

			Eigen::Vector3f frontMid;
			frontMid << min[0] + i * cellWidth + cellWidth / 2., min[1], min[2] + j * cellHeight + cellHeight / 2.;
			Eigen::Vector3f backMid;
			backMid << min[0] + i * cellWidth + cellWidth / 2., max[1], min[2] + j * cellHeight + cellHeight / 2.;

			for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
				for (auto fit = (*rit)->faces.begin(); fit != (*rit)->faces.end(); fit++) {
					Eigen::Vector3f intersect;
					if (RayIntersectsTriangle(frontMid, backMid, (*fit), intersect)) {
						float dist = (intersect - frontMid).norm();
						if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
							points[0][i][j].vertex = new Data::Vertex;
							points[0][i][j].vertex->pos[0] = intersect[0];
							points[0][i][j].vertex->pos[1] = intersect[1];
							points[0][i][j].vertex->pos[2] = intersect[2];
							points[0][i][j].dist = dist;
						}

						if (! points[1][i][j].vertex || dist > points[1][i][j].dist) {
							points[1][i][j].vertex = new Data::Vertex;
							points[1][i][j].vertex->pos[0] = intersect[0];
							points[1][i][j].vertex->pos[1] = intersect[1];
							points[1][i][j].vertex->pos[2] = intersect[2];
							points[1][i][j].dist = dist;
						}

					}
				}
			}
		}
	}

	return points;
}

HullGridRays::HullGridPoint*** HullGridRays::findXYHullPoints(Data::Part* part, int n, int m)
{
	HullGridRays::HullGridPoint*** points = new HullGridRays::HullGridPoint**[2]; // back and front
	points[0] = new HullGridRays::HullGridPoint*[n];
	points[1] = new HullGridRays::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridRays::HullGridPoint[m]();
		points[1][i] = new HullGridRays::HullGridPoint[m]();
	}

	// XY plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[0] - min[0];
	float height = max[1] - min[1];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			points[0][i][j].dist = 1000.;
			points[1][i][j].dist = 0.;

			Eigen::Vector3f frontMid;
			frontMid << min[0] + i * cellWidth + cellWidth / 2., min[1] + j * cellHeight + cellHeight / 2., min[2];
			Eigen::Vector3f backMid;
			backMid << min[0] + i * cellWidth + cellWidth / 2., min[1] + j * cellHeight + cellHeight / 2., max[2];

			for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
				for (auto fit = (*rit)->faces.begin(); fit != (*rit)->faces.end(); fit++) {
					Eigen::Vector3f intersect;
					if (RayIntersectsTriangle(frontMid, backMid, (*fit), intersect)) {
						float dist = (intersect - frontMid).norm();
						if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
							points[0][i][j].vertex = new Data::Vertex;
							points[0][i][j].vertex->pos[0] = intersect[0];
							points[0][i][j].vertex->pos[1] = intersect[1];
							points[0][i][j].vertex->pos[2] = intersect[2];
							points[0][i][j].dist = dist;
						}

						if (! points[1][i][j].vertex || dist > points[1][i][j].dist) {
							points[1][i][j].vertex = new Data::Vertex;
							points[1][i][j].vertex->pos[0] = intersect[0];
							points[1][i][j].vertex->pos[1] = intersect[1];
							points[1][i][j].vertex->pos[2] = intersect[2];
							points[1][i][j].dist = dist;
						}

					}
				}
			}
		}
	}

	return points;
}

HullGridRays::HullGridPoint*** HullGridRays::findYZHullPoints(Data::Part* part, int n, int m)
{
	HullGridRays::HullGridPoint*** points = new HullGridRays::HullGridPoint**[2]; // back and front
	points[0] = new HullGridRays::HullGridPoint*[n];
	points[1] = new HullGridRays::HullGridPoint*[n];
	for (int i=0; i<n; i++) {
		points[0][i] = new HullGridRays::HullGridPoint[m]();
		points[1][i] = new HullGridRays::HullGridPoint[m]();
	}

	// YZ plane
	Eigen::Vector3f min = part->boundingBox.min();
	Eigen::Vector3f max = part->boundingBox.max();
	float width = max[1] - min[1];
	float height = max[2] - min[2];

	float cellWidth = width / n;
	float cellHeight = height / m;

	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			points[0][i][j].dist = 1000.;
			points[1][i][j].dist = 0.;

			Eigen::Vector3f frontMid;
			frontMid << min[0], min[1] + i * cellWidth + cellWidth / 2., min[2] + j * cellHeight + cellHeight / 2.;
			Eigen::Vector3f backMid;
			backMid << max[0], min[1] + i * cellWidth + cellWidth / 2., min[2] + j * cellHeight + cellHeight / 2.;

			for (auto rit = part->regions.begin(); rit != part->regions.end(); rit++) {
				for (auto fit = (*rit)->faces.begin(); fit != (*rit)->faces.end(); fit++) {
					Eigen::Vector3f intersect;
					if (RayIntersectsTriangle(frontMid, backMid, (*fit), intersect)) {
						float dist = (intersect - frontMid).norm();
						if (! points[0][i][j].vertex || dist < points[0][i][j].dist) {
							points[0][i][j].vertex = new Data::Vertex;
							points[0][i][j].vertex->pos[0] = intersect[0];
							points[0][i][j].vertex->pos[1] = intersect[1];
							points[0][i][j].vertex->pos[2] = intersect[2];
							points[0][i][j].dist = dist;
						}

						if (! points[1][i][j].vertex || dist > points[1][i][j].dist) {
							points[1][i][j].vertex = new Data::Vertex;
							points[1][i][j].vertex->pos[0] = intersect[0];
							points[1][i][j].vertex->pos[1] = intersect[1];
							points[1][i][j].vertex->pos[2] = intersect[2];
							points[1][i][j].dist = dist;
						}

					}
				}
			}
		}
	}

	return points;
}

bool HullGridRays::RayIntersectsTriangle(Eigen::Vector3f rayOrigin, 
                           Eigen::Vector3f rayTarget, 
                           Data::Face* face,
                           Eigen::Vector3f& outIntersectionPoint)
{
	const float EPSILON = 0.0000001; 
	Eigen::Vector3f rayVector = rayTarget - rayOrigin;
	float dist = rayVector.norm();
	// rayVector[0] /= dist;
	// rayVector[1] /= dist;
	// rayVector[2] /= dist;
	Eigen::Vector3f vertex1 = face->v1->pos.head<3>();
	Eigen::Vector3f vertex2 = face->v2->pos.head<3>();
	Eigen::Vector3f vertex3 = face->v3->pos.head<3>();
	Eigen::Vector3f edge1, edge2, h, s, q;
	float a,f,u,v;
	edge1 = vertex2 - vertex1;
	edge2 = vertex3 - vertex1;
	// h = rayVector.crossProduct(edge2);
	h = rayVector.cross(edge2);
	// h = face->normal.head<3>();
	// a = edge1.dotProduct(h);
	a = edge1.dot(h);
	if (a > -EPSILON && a < EPSILON) {
		return false;
	}
	f = 1/a;
	s = rayOrigin - vertex1;
	// u = f * (s.dotProduct(h));
	u = f * (s.dot(h));
	if (u < 0.0 || u > 1.0)
		return false;
	// q = s.crossProduct(edge1);
	q = s.cross(edge1);
	// v = f * rayVector.dotProduct(q);
	v = f * rayVector.dot(q);

	if (v < 0.0 || u + v > 1.0)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	// float t = f * edge2.dotProduct(q);
	float t = f * edge2.dot(q);
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t; 
		return true;
	} else { // This means that there is a line intersection but not a ray intersection.
		return false;
	}
}
