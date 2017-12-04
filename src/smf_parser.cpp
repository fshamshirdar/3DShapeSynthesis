#include "smf_parser.h"

SMFParser::SMFParser()
{
}

SMFParser::~SMFParser() { }

WingedEdge::W_edge* SMFParser::findEdge(const WingedEdge::Region* region, const WingedEdge::Vertex* v1, const WingedEdge::Vertex* v2)
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

void SMFParser::calculateFaceNormal(WingedEdge::Face* face)
{
	float ux = face->v2->pos.x() - face->v1->pos.x();
	float uy = face->v2->pos.y() - face->v1->pos.y();
	float uz = face->v2->pos.z() - face->v1->pos.z();

	float vx = face->v3->pos.x() - face->v1->pos.x();
	float vy = face->v3->pos.y() - face->v1->pos.y();
	float vz = face->v3->pos.z() - face->v1->pos.z();

	float nx = (uy * vz) - (uz * vy);
	float ny = (uz * vx) - (ux * vz);
	float nz = (ux * vy) - (uy * vx);
	GLfloat length = sqrt(nx*nx + ny*ny + nz*nz);
	if (length < 1e-7) {
		face->normal[0] = face->normal[1] = face->normal[2] = face->normal[3] = 0.0;
		return;
	} else {
		face->normal[0] = nx / length;
		face->normal[1] = ny / length;
		face->normal[2] = nz / length;
		face->normal[3] =
			- face->normal.x() * face->v1->pos.x()
			- face->normal.y() * face->v1->pos.y()
			- face->normal.z() * face->v1->pos.z();
	}

}

void SMFParser::calculateVertexNormal(WingedEdge::Vertex* vertex)
{
	float nx = 0.0, ny = 0.0, nz = 0.0;
	int counter = 0;

	vertex->Q = Eigen::Matrix4f::Zero();
	WingedEdge::W_edge *e0 = vertex->edge;
	WingedEdge::W_edge *edge = e0;
	do {
		if (! edge) {
			std::cout << "break" << std::endl;
			break;
		}
		// std::cout << edge->start->id+1 << " " << edge->end->id+1 << std::endl;
		if (edge->end == vertex) {
			if (edge->right) {
				nx += edge->right->normal.x();
				ny += edge->right->normal.y();
				nz += edge->right->normal.z();
				vertex->Q += edge->right->normal * edge->right->normal.transpose();
				counter++;
			}
			edge = edge->right_next;
		} else {
			if (edge->left) {
				nx += edge->left->normal.x();
				ny += edge->left->normal.y();
				nz += edge->left->normal.z();
				vertex->Q += edge->left->normal * edge->left->normal.transpose();
				counter++;
			}
			edge = edge->left_next;
		}
	} while(edge != e0);

	nx /= counter;
	ny /= counter;
	nz /= counter;

	vertex->normal[0] = nx;
	vertex->normal[1] = ny;
	vertex->normal[2] = nz;
}

bool SMFParser::load(const std::string &filename)
{
	std::ifstream infile(filename.c_str());
	if (! infile.good()) {
		return false;
	}

	bool size_parsed = false;
	std::string line;
	int i = 0; // total vertex number
	int j = 0;
	int k = 0;

	float minDistance = 100.0;

	WingedEdge::Region* currentRegion = new WingedEdge::Region;
	WingedEdge::Region* previousRegion;

	// load line by line
	while (std::getline(infile, line)) {
		if (line.size() < 1) {
			continue;
		}
		std::istringstream iss(line.substr(1));
		// switch by the leading character
		switch (line[0])
		{
		case 'v':
			{
			float x, y, z;
			iss >> x >> y >> z;
			WingedEdge::Vertex* newVertex = new WingedEdge::Vertex;
			newVertex->id = i;
			newVertex->regionId = k;
			newVertex->pos[0] = x;
			newVertex->pos[1] = y;
			newVertex->pos[2] = z;
			newVertex->pos[3] = 1;
			currentRegion->vertices.push_back(newVertex); // TODO: modify smf file to store number of vertices as well
			i++;
			}
			break;
		case 'g':
			currentRegion->id = k;
			currentRegion->name = line.substr(2);
			regions.push_back(currentRegion);
			k++;
			previousRegion = currentRegion;
			currentRegion = new WingedEdge::Region;
			std::cout << previousRegion->name << std::endl;
			break;
		case 'f':
			{
			bool e1f = false, e2f = false, e3f = false;
			bool e1e = true, e2e = true, e3e = true;
			int v1, v2, v3;
			int ov1, ov2, ov3;
			iss >> v1 >> v2 >> v3;
			v1 --; v2 --; v3 --;

			v1 -= (i - previousRegion->vertices.size());
			v2 -= (i - previousRegion->vertices.size());
			v3 -= (i - previousRegion->vertices.size());

			ov1 = v1;
			ov2 = v2;
			ov3 = v3;
			WingedEdge::Face* newFace = new WingedEdge::Face;

			if (v1 > v2) {
				ov2 = v1;
				ov1 = v2;
				e1f = true;
			}
			WingedEdge::W_edge* e1 = findEdge(previousRegion, previousRegion->vertices[ov1], previousRegion->vertices[ov2]);
			if (! e1) {
				e1e = false;
				e1 = new WingedEdge::W_edge;
				e1->start = previousRegion->vertices[ov1];
				e1->end = previousRegion->vertices[ov2];
				e1->left = NULL;
				e1->right = NULL;
				e1->left_next = NULL;
				e1->left_prev = NULL;
				e1->right_next = NULL;
				e1->right_prev = NULL;

				previousRegion->edges.push_back(e1);
			}

			ov1 = v1;
			ov2 = v2;
			ov3 = v3;
			if (v2 > v3) {
				ov3 = v2;
				ov2 = v3;
				e2f = true;
			}
			WingedEdge::W_edge* e2 = findEdge(previousRegion, previousRegion->vertices[ov2], previousRegion->vertices[ov3]);
			if (! e2) {
				e2e = false;
				e2 = new WingedEdge::W_edge;
				e2->start = previousRegion->vertices[ov2];
				e2->end = previousRegion->vertices[ov3];
				e2->left = NULL;
				e2->right = NULL;
				e2->left_next = NULL;
				e2->left_prev = NULL;
				e2->right_next = NULL;
				e2->right_prev = NULL;

				previousRegion->edges.push_back(e2);
			}

			ov1 = v1;
			ov2 = v2;
			ov3 = v3;
			if (v3 > v1) {
				ov3 = v1;
				ov1 = v3;
				e3f = true;
			}
			WingedEdge::W_edge* e3 = findEdge(previousRegion, previousRegion->vertices[ov3], previousRegion->vertices[ov1]);
			if (! e3) {
				e3e = false;
				e3 = new WingedEdge::W_edge;
				e3->start = previousRegion->vertices[ov3];
				e3->end = previousRegion->vertices[ov1];
				e3->left = NULL;
				e3->right = NULL;
				e3->left_next = NULL;
				e3->left_prev = NULL;
				e3->right_next = NULL;
				e3->right_prev = NULL;

				previousRegion->edges.push_back(e3);
			}

			if (e1f) {
				e1->right = newFace;
				e1->right_prev = e2;
				e1->right_next = e3;
			} else {
				e1->left = newFace;
				e1->left_prev = e2;
				e1->left_next = e3;
			}

			if (e2f) {
				e2->right = newFace;
				e2->right_prev = e3;
				e2->right_next = e1;
			} else {
				e2->left = newFace;
				e2->left_prev = e3;
				e2->left_next = e1;
			}

			if (e3f) {
				e3->right = newFace;
				e3->right_prev = e1;
				e3->right_next = e2;
			} else {
				e3->left = newFace;
				e3->left_prev = e1;
				e3->left_next = e2;
			}

			previousRegion->vertices[v1]->edge = e1;
			previousRegion->vertices[v2]->edge = e2;
			previousRegion->vertices[v3]->edge = e3;

	//		previousRegion->vertices[v1]->pairs.push_back(vertices[v2]);
	//		previousRegion->vertices[v2]->pairs.push_back(vertices[v3]);
	//		previousRegion->vertices[v3]->pairs.push_back(vertices[v1]);

			newFace->id = j;
			newFace->regionId = previousRegion->id;
			newFace->edge = e1;
			newFace->v1 = previousRegion->vertices[v1];
			newFace->v2 = previousRegion->vertices[v2];
			newFace->v3 = previousRegion->vertices[v3];
			calculateFaceNormal(newFace);
			previousRegion->faces.push_back(newFace);
			j++;
			}
			break;
		case '#':
			if (size_parsed == false || false) { // TODO: text format required, off for now
				iss >> vertices_size >> faces_size;
				size_parsed = true;
				currentRegion->vertices.resize(vertices_size);
				currentRegion->faces.resize(faces_size);
			}
			break;
		default:
			break;
		}
	}
	infile.close();
	std::cout << "loadFile " << filename << std::endl;
	for (int i = 0; i < regions.size(); i++) {
		for (int j = 0; j < regions[i]->vertices.size(); j++) {
			calculateVertexNormal(regions[i]->vertices[j]);
		}
	}

	WingedEdge::Vertex* vertex;
	WingedEdge::Face* face;
	for (int i = 0; i < regions.size(); i++) {
		for (int j = 0; j < regions.size(); j++) {
			if (i != j && std::find(regions[i]->neighbors.begin(), regions[i]->neighbors.end(), regions[j]) == regions[i]->neighbors.end()) {
				for (int k = 0; k < regions[i]->vertices.size(); k++) {
					vertex = regions[i]->vertices[k];
					for (int l = 0; l < regions[j]->faces.size(); l++) {
						face = regions[j]->faces[l];
						float dist = (vertex->pos - face->v1->pos).dot(face->normal);
						Eigen::Vector4f point = vertex->pos - (dist * face->normal);
						Eigen::Vector3f point3 = point.head<3>();
						if (isPointWithinTriangle(face, point3)) {
							if (fabs(dist) < 0.001) {
								regions[i]->neighbors.push_back(regions[j]);
								regions[j]->neighbors.push_back(regions[i]);

								l = regions[j]->faces.size();
								k = regions[i]->faces.size();
							}
						}
					}
				}
			}
		}
	}

	for (int i = 0; i < regions[1]->neighbors.size(); i++) {
		std::cout << regions[1]->neighbors[i]->name << std::endl;
	}



	// parse edge list and face map
	return true;
}


bool SMFParser::isPointWithinTriangle(WingedEdge::Face* face, Eigen::Vector3f P)
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

bool SMFParser::sameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f A, Eigen::Vector3f B)
{
	Eigen::Vector3f cp1 = (B - A).cross(p1 - A);
	Eigen::Vector3f cp2 = (B - A).cross(p2 - A);
	return (cp1.dot(cp2) >= 0);
}

WingedEdge::Vertex* SMFParser::getMidPoint(const WingedEdge::Vertex* v1, const WingedEdge::Vertex* v2) {
	WingedEdge::Vertex* m = new WingedEdge::Vertex;
	m->Q = v1->Q + v2->Q;

	Eigen::Matrix4f tmp = m->Q;
	tmp(3, 0) = 0.;
	tmp(3, 1) = 0.;
	tmp(3, 2) = 0.;
	tmp(3, 3) = 1.;

	Eigen::Vector4f I(0, 0, 0, 1);
	if (tmp.determinant() > 1e-7) {
		m->pos = tmp.inverse() * I;
	} else {
		m->pos = (v1->pos + v2->pos) / 2.0;
	}
	m->pos /= m->pos[3]; // TODO: check
	m->edge = v1->edge;
//	m->pos[3] = 1;
	return m;
}
