#include "data.h"
#include "smf_parser.h"
#include <iostream>
#include <fstream>

void Data::save()
{
	std::ofstream file;
	file.open(path);
	int id = 0;
	for ( auto pit = parts.begin(); pit != parts.end(); pit++ ) {
		if ((*pit)->type >= Data::Part::Type::LEG) { // do not store virtual parts
			continue;
		}

		for(auto rit = (*pit)->regions.begin(); rit != (*pit)->regions.end(); rit++ ) {
			for(auto vit = (*rit)->vertices.begin(); vit != (*rit)->vertices.end(); vit++ ) { 
				file << "v " << (*vit)->pos[0] << " " << (*vit)->pos[1] << " " <<(*vit)->pos[2] << std::endl;
				(*vit)->id = id;
				id ++;
			}
			file << "g " << (*rit)->name << " " << (*pit)->type << std::endl;
			for(auto fit = (*rit)->faces.begin(); fit != (*rit)->faces.end(); fit ++) {
				file << "f " << (*fit)->v1->id+1 << " " << (*fit)->v2->id+1 << " " << (*fit)->v3->id+1 << std::endl;
			}  
		}
	} 
	file.close();
}

Data* Data::clone()
{
	SMFParser* parser = new SMFParser();
	Data* newData = parser->load(path);
	return newData;
}

Data::Part* Data::findPartByType(Data::Part::Type type)
{
	for (auto it = parts.begin() ; it != parts.end(); ++it) {
		if ((*it)->type == type) {
			return (*it);
		}
	}

	Data::Part* part = new Data::Part();
	part->parent = this;
	part->type = type;

	if (type == Data::Part::Type::LEG) {
		part->type = Data::Part::Type::TWO_LEGGED;
		Eigen::AlignedBox3f boundingBox;
		for (int i = Data::Part::Type::LEG_FRONT_SPINDLE; i <= Data::Part::Type::LEG_RIGHT_SPINDLE; i++) {
			Data::Part* child = findPartByType((Data::Part::Type)(i));
			if (child->regions.size() == 0) {
				continue;
			}

			if ((Data::Part::Type)(i) == Data::Part::Type::LEG_FRONT_LEG) {
				part->type = Data::Part::Type::FOUR_LEGGED;
			} else if ((Data::Part::Type)(i) == Data::Part::Type::LEG_BAR) {
				part->type = Data::Part::Type::SINGLE_LEGGED;
			}
			boundingBox = boundingBox.extend(child->boundingBox);
			part->regions.insert(part->regions.end(), child->regions.begin(), child->regions.end());
		}
		part->boundingBox = boundingBox;
	}

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
	if (part->type == Data::Part::Type::FOUR_LEGGED ||
	    part->type == Data::Part::Type::SINGLE_LEGGED ||
	    part->type == Data::Part::Type::TWO_LEGGED) {
		for (int i = Data::Part::Type::LEG_FRONT_SPINDLE; i <= Data::Part::Type::LEG_RIGHT_SPINDLE; i++) {
			Data::Part* targetPart = part->parent->findPartByType((Data::Part::Type)(i));
			if (targetPart->regions.size() == 0) {
				deletePartByType((Data::Part::Type)(i));
				continue;
			}
			replacePartByType(targetPart);
		}
		deletePartByType(Data::Part::Type::FOUR_LEGGED);
		deletePartByType(Data::Part::Type::SINGLE_LEGGED);
		deletePartByType(Data::Part::Type::TWO_LEGGED);
	} else {
		for (int i=0; i < parts.size(); i++) {
			if (parts[i]->type == part->type) {
				parts[i] = part;
				return;
			}
		}
	}

	//findPartByType(part->type);
	//replacePartByType(part);
}

void Data::deletePartByType(Data::Part::Type type)
{
	for (auto it=parts.begin(); it != parts.end();) {
		if ((*it)->type == type) {
			it = parts.erase(it);
		} else {
			it ++;
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
	for (auto p1it = parts.begin(); p1it != parts.end(); p1it++) {
		for (auto p2it = parts.begin(); p2it != parts.end(); p2it++) {
			if ((*p1it) != (*p2it) &&
			    ! (*p1it)->isChild(*p2it) &&
			    ! (*p1it)->isParent(*p2it)) {
				findRegionsNeighborsByVertexToFaceDistance(*p1it, *p2it);
			}
		}
	}
}

void Data::findPartsNeighborsByVertexToFaceDistanceForPart(Data::Part* part)
{
	for (auto p2it = parts.begin(); p2it != parts.end(); p2it++) {
		if (part != (*p2it) &&
		    ! (part)->isChild(*p2it) &&
		    ! (part)->isParent(*p2it)) {
			findRegionsNeighborsByVertexToFaceDistance(part, *p2it);
			findRegionsNeighborsByVertexToFaceDistance(*p2it, part);
		}
	}
}

void Data::findRegionsNeighborsByVertexToFaceDistance(Data::Part* part1, Data::Part* part2)
{
	Data::Vertex* vertex;
	Data::Face* face;
	for (int i = 0; i < part1->regions.size(); i++) {
		for (int k = 0; k < part1->regions[i]->vertices.size(); k++) {
			vertex = part1->regions[i]->vertices[k];
			for (int j = 0; j < part2->regions.size(); j++) {
				for (int l = 0; l < part2->regions[j]->faces.size(); l++) {
					face = part2->regions[j]->faces[l];
					float dist = (vertex->pos - face->v1->pos).dot(face->normal);
					Eigen::Vector4f point = vertex->pos - (dist * face->normal);
					Eigen::Vector3f point3 = point.head<3>();
					if (isPointWithinTriangle(face, point3)) {
						if (fabs(dist) < 0.005) {
							part1->addVertexToPartIntersection(part2, vertex, true);
							part2->addVertexToPartIntersection(part1, vertex, false);

							// l = part2->regions[j]->faces.size();
							j = part2->regions.size();
							break;
						}
					}
				}
			}
		}
	}
}

void Data::findPartsNeighborsByBoxIntersection()
{
	for (auto p1it = parts.begin(); p1it != parts.end(); p1it++) {
		for (auto p2it = parts.begin(); p2it != parts.end(); p2it++) {
			if ((*p1it) != (*p2it) &&
			    ! (*p1it)->isChild(*p2it) &&
			    ! (*p1it)->isParent(*p2it)) {
				findRegionsNeighborsByBoxIntersection(*p1it, *p2it);
			}
		}
	}
}

void Data::findPartsNeighborsByBoxIntersectionForPart(Data::Part* part)
{
	for (auto p2it = parts.begin(); p2it != parts.end(); p2it++) {
		if (part != (*p2it) &&
		    ! part->isChild(*p2it) &&
		    ! part->isParent(*p2it)) {
			findRegionsNeighborsByBoxIntersection(part, *p2it);
		}
	}
}

void Data::findRegionsNeighborsByBoxIntersection(Data::Part* part1, Data::Part* part2)
{
	float scale = 1.;
	Eigen::Vector3f translation;
	translation << .015, .015, .015;

	Eigen::AlignedBox3f intersection = part1->boundingBox.intersection(part2->boundingBox);
	Eigen::Vector3f min = intersection.min();
	Eigen::Vector3f max = intersection.max();
	Eigen::Vector3f mid = (max + min) / 2.;
	Eigen::Vector3f newMin = (min - mid) * scale + mid;
	Eigen::Vector3f newMax = (max - mid) * scale + mid;
	newMin = (newMin - mid) - translation + mid;
	newMax = (newMax - mid) + translation + mid;
	intersection = Eigen::AlignedBox3f(newMin, newMax);

	for (auto r1it = part1->regions.begin(); r1it != part1->regions.end(); r1it++) {
		for (auto r2it = part2->regions.begin(); r2it != part2->regions.end(); r2it++) {
			Eigen::AlignedBox3f intersection = (*r1it)->boundingBox.intersection((*r2it)->boundingBox);
			Eigen::Vector3f min = intersection.min();
			Eigen::Vector3f max = intersection.max();
			Eigen::Vector3f mid = (max + min) / 2.;
			Eigen::Vector3f newMin = (min - mid) * scale + mid;
			Eigen::Vector3f newMax = (max - mid) * scale + mid;
			newMin = (newMin - mid) - translation + mid;
			newMax = (newMax - mid) + translation + mid;
			intersection = Eigen::AlignedBox3f(newMin, newMax);

			if (intersection.isEmpty()) {
				continue;
			}

			for (auto r1vit = (*r1it)->vertices.begin(); r1vit != (*r1it)->vertices.end(); r1vit++) {
				Eigen::Vector4f pos = (*r1vit)->pos;
				if (pos[0] > newMin[0] && pos[1] > newMin[1] && pos[2] > newMin[2] &&
						pos[0] < newMax[0] && pos[1] < newMax[1] && pos[2] < newMax[2]) {
					for (auto r2fit = (*r2it)->faces.begin(); r2fit != (*r2it)->faces.end(); r2fit++) {
						float dist = (pos - (*r2fit)->v1->pos).dot((*r2fit)->normal);
						Eigen::Vector4f point = pos - (dist * (*r2fit)->normal);
						Eigen::Vector3f point3 = point.head<3>();
						if (isPointWithinTriangle((*r2fit), point3)) {
							if (fabs(dist) < 0.015) {
								part1->addVertexToPartIntersection(part2, (*r1vit), true);
								part2->addVertexToPartIntersection(part1, (*r1vit), false);

								break;
							}
						}
					}

//					part1->addVertexToPartIntersection(part2, (*r1vit), true);
//					part2->addVertexToPartIntersection(part1, (*r1vit), false);
				}
			}
			for (auto r2vit = (*r2it)->vertices.begin(); r2vit != (*r2it)->vertices.end(); r2vit++) {
				Eigen::Vector4f pos = (*r2vit)->pos;
				if (pos[0] > newMin[0] && pos[1] > newMin[1] && pos[2] > newMin[2] &&
						pos[0] < newMax[0] && pos[1] < newMax[1] && pos[2] < newMax[2]) {
					for (auto r1fit = (*r1it)->faces.begin(); r1fit != (*r1it)->faces.end(); r1fit++) {
						float dist = (pos - (*r1fit)->v1->pos).dot((*r1fit)->normal);
						Eigen::Vector4f point = pos - (dist * (*r1fit)->normal);
						Eigen::Vector3f point3 = point.head<3>();
						if (isPointWithinTriangle((*r1fit), point3)) {
							if (fabs(dist) < 0.015) {
								part2->addVertexToPartIntersection(part1, (*r2vit), true);
								part1->addVertexToPartIntersection(part2, (*r2vit), false);

								break;
							}
						}
					}

//					part2->addVertexToPartIntersection(part1, (*r2vit), true);
//					part1->addVertexToPartIntersection(part2, (*r2vit), false);
				}
			}
			/*
			   for (auto r1it = part1->regions.begin(); r1it != part1->regions.end(); r1it++) {
			   for (auto r1fit = (*r1it)->faces.begin(); r1fit != (*r1it)->faces.end(); r1fit++) {
			   float dist = (pos - (*r1fit)->v1->pos).dot((*r1fit)->normal);
			   Eigen::Vector4f point = pos - (dist * (*r1fit)->normal);
			   Eigen::Vector3f point3 = point.head<3>();
			   if (isPointWithinTriangle((*r1fit), point3)) {
			   if (fabs(dist) < 0.005) {
			   part1->addVertexToPartIntersection(part2, (*r2vit), false);
			   part2->addVertexToPartIntersection(part1, (*r2vit), true);

			   r1it = part1->regions.end();
			   break;
			   }
			   }
			   }
			   }
			 */

			//				part2->addVertexToPartIntersection(part1, (*r2vit), true);
			//				part1->addVertexToPartIntersection(part2, (*r2vit), false);
		}
	}
}
