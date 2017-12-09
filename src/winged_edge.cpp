#include "data.h"

Data::Part* Data::findPartByType(Data::Part::Type type)
{
	for (auto it = parts.begin() ; it != parts.end(); ++it) {
		if ((*it)->type == type) {
			return (*it);
		}
	}

	Data::Part* part = new Data::Part();
	part->type = type;

	parts.push_back(part);

	return part;
}

void Data::addPart(Data::Part* part)
{
	parts.push_back(part);
}

void Data::addParts(Data* data)
{
	for (auto it = data->parts.begin(); it != data->parts.end(); it++) {
		parts.push_back((*it));
	}
}

void Data::replacePartByType(Data::Part* part)
{
	for (int i=0; i < parts.size(); i++) {
		if (parts[i]->type == part->type) {
			parts[i] = part;
		}
	}
}

Data::W_edge* Data::findEdge(const Data::Region* region, const Data::Vertex* v1, const Data::Vertex* v2)
{
	for (int i = 0; i < region->edges.size(); i++) {
		if (region->edges.at(i)->start == v1 && region->edges.at(i)->end == v2) {
			return region->edges.at(i);
		}

		if (region->edges.at(i)->end == v1 && region->edges.at(i)->start == v2) {
			return region->edges.at(i);
		}
	}

	return NULL;
}

bool Data::isPointWithinTriangle(Data::Face* face, Eigen::Vector3f P)
{
	Eigen::Vector3f A = face->v1->pos.head<3>();
	Eigen::Vector3f B = face->v2->pos.head<3>();
	Eigen::Vector3f C = face->v3->pos.head<3>();
	if (sameSide(P, A, B, C) && sameSide(P, B, A, C) && sameSide(P, C, A, B)) {
		Eigen::Vector3f dc1 = A - B;
		Eigen::Vector3f dc2 = A - C;
		Eigen::Vector3f vc1 = dc1.cross(dc2);
		if (fabs((A - P).dot(vc1)) <= .01f) {
			return true;
		}
	}

	return false;
}

bool Data::sameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f A, Eigen::Vector3f B)
{
	Eigen::Vector3f cp1 = (B - A).cross(p1 - A);
	Eigen::Vector3f cp2 = (B - A).cross(p2 - A);
	return (cp1.dot(cp2) >= 0);
}

void Data::findPartsNeighborsByVertexToFaceDistance()
{
	Data::Vertex* vertex;
	Data::Face* face;
	for (auto p1it = parts.begin(); p1it != parts.end(); p1it++) {
		for (auto p2it = parts.begin(); p2it != parts.end(); p2it++) {
			
		}

	}
}

void Data::findRegionsNeighborsByVertexToFaceDistance(Data::Part* part1, Data::Part* part2)
{
	for (int i = 0; i < part1->regions.size(); i++) {
		for (int j = 0; j < part2->regions.size(); j++) {
			for (int k = 0; k < part1->regions[i]->vertices.size(); k++) {
				vertex = part1->regions[i]->vertices[k];
				for (int l = 0; l < part2->regions[j]->faces.size(); l++) {
					face = part2->regions[j]->faces[l];
					float dist = (vertex->pos - face->v1->pos).dot(face->normal);
					Eigen::Vector4f point = vertex->pos - (dist * face->normal);
					Eigen::Vector3f point3 = point.head<3>();
					if (isPointWithinTriangle(face, point3)) {
						if (fabs(dist) < 0.001) {
						//	regions[i]->neighbors.push_back(regions[j]);
						//	regions[j]->neighbors.push_back(regions[i]);

							part1->neighbors.push_back();
							part2->neighbors.push_back();

							l = regions[j]->faces.size();
							k = regions[i]->faces.size();
						}
					}
				}
			}
		}
	}
}
