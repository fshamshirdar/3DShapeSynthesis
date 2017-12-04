#ifndef __SMF_PARSER_H__
#define __SMF_PARSER_H__

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <vector>
#include <set>
#include "data.h"

class SMFParser {
public:
	SMFParser();
	~SMFParser();
	void calculateFaceNormal(WingedEdge::Face* face);
	void calculateVertexNormal(WingedEdge::Vertex* vertex);
	bool load(const std::string &filename);
	void simplify(int k);
	WingedEdge::Vertex* getMidPoint(const WingedEdge::Vertex* v1, const WingedEdge::Vertex* v2);
	WingedEdge::W_edge* findEdge(const WingedEdge::Region* region, const WingedEdge::Vertex* v1, const WingedEdge::Vertex* v2);
	bool isPointWithinTriangle(WingedEdge::Face* face, Eigen::Vector3f P);
	bool sameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f A, Eigen::Vector3f B);

public:
	std::vector<WingedEdge::Region*> regions;

	int vertices_size;
	int faces_size;
};

#endif
