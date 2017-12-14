#include "smf_parser.h"

SMFParser::SMFParser()
{
}

SMFParser::~SMFParser() { }

Data* SMFParser::load(const std::string &filename)
{
	Data* data = new Data();
	data->path = filename;

	std::ifstream infile(filename.c_str());
	if (! infile.good()) {
		return NULL;
	}

	bool size_parsed = false;
	std::string line;
	int p = 0;
	int i = 0; // total vertex number
	int j = 0;
	int k = 0;
	int partType;

	float minDistance = 100.0;

	Data::Part* currentPart = NULL;
	Data::Region* currentRegion = new Data::Region();
	Data::Region* previousRegion;

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
			Data::Vertex* newVertex = new Data::Vertex;
			newVertex->id = i;
			newVertex->regionId = k;
			newVertex->pos[0] = x;
			newVertex->pos[1] = y;
			newVertex->pos[2] = z;
			newVertex->pos[3] = 1;

			if (x < currentRegion->boundingBox.min()[0]) {
				currentRegion->boundingBox.min()[0] = x;
			}
			if (y < currentRegion->boundingBox.min()[1]) {
				currentRegion->boundingBox.min()[1] = y;
			}
			if (z < currentRegion->boundingBox.min()[2]) {
				currentRegion->boundingBox.min()[2] = z;
			}
			if (x > currentRegion->boundingBox.max()[0]) {
				currentRegion->boundingBox.max()[0] = x;
			}
			if (y > currentRegion->boundingBox.max()[1]) {
				currentRegion->boundingBox.max()[1] = y;
			}
			if (z > currentRegion->boundingBox.max()[2]) {
				currentRegion->boundingBox.max()[2] = z;
			}

			currentRegion->vertices.push_back(newVertex); // TODO: modify smf file to store number of vertices as well
			i++;
			}
			break;
		case 'g':
			currentRegion->id = k;
			// currentRegion->name = line.substr(2);
			iss >> currentRegion->name >> partType;
			currentPart = data->findPartByType(static_cast<Data::Part::Type>(partType));
			currentPart->regions.push_back(currentRegion);

			for (int bi = 0; bi < 3; bi++) {
				if (currentRegion->boundingBox.min()[bi] < currentPart->boundingBox.min()[bi]) {
					currentPart->boundingBox.min()[bi] = currentRegion->boundingBox.min()[bi];
				}

				if (currentRegion->boundingBox.max()[bi] > currentPart->boundingBox.max()[bi]) {
					currentPart->boundingBox.max()[bi] = currentRegion->boundingBox.max()[bi];
				}
			}

			k++;
			previousRegion = currentRegion;
			currentRegion = new Data::Region();
			std::cout << partType << " " << previousRegion->name << std::endl;
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
			Data::Face* newFace = new Data::Face;

			if (v1 > v2) {
				ov2 = v1;
				ov1 = v2;
				e1f = true;
			}
			Data::W_edge* e1 = data->findEdge(previousRegion, previousRegion->vertices[ov1], previousRegion->vertices[ov2]);
			if (! e1) {
				e1e = false;
				e1 = new Data::W_edge;
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
			Data::W_edge* e2 = data->findEdge(previousRegion, previousRegion->vertices[ov2], previousRegion->vertices[ov3]);
			if (! e2) {
				e2e = false;
				e2 = new Data::W_edge;
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
			Data::W_edge* e3 = data->findEdge(previousRegion, previousRegion->vertices[ov3], previousRegion->vertices[ov1]);
			if (! e3) {
				e3e = false;
				e3 = new Data::W_edge;
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
			newFace->calculateFaceNormal();
			previousRegion->faces.push_back(newFace);
			j++;
			}
			break;
		case '#':
			if (size_parsed == false || false) { // TODO: text format required, off for now
				iss >> data->vertices_size >> data->faces_size;
				size_parsed = true;
				currentRegion->vertices.resize(data->vertices_size);
				currentRegion->faces.resize(data->faces_size);
			}
			break;
		default:
			break;
		}
	}
	infile.close();
	std::cout << "loadFile " << filename << std::endl;
	for (int i = 0; i < data->parts.size(); i++) {
		for (int j = 0; j < data->parts[i]->regions.size(); j++) {
			for (int k = 0; k < data->parts[i]->regions[j]->vertices.size(); k++) {
				data->parts[i]->regions[j]->vertices[k]->calculateVertexNormal();
			}
		}
	}

	// parse edge list and face map
	return data;
}
