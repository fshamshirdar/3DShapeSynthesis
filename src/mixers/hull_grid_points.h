#pragma once

#include "data.h"
#include "mix_match.h"
#include <iostream>

class HullGridPoints: public MixMatch {
public:
	class HullGridPoint;
	class HullVertexPair;

	Data* mix(Data* chair1, Data* chair2);
	std::vector<HullGridPoints::HullVertexPair*> findCorrespondingPoints(HullGridPoints::HullGridPoint*** refPoints, HullGridPoints::HullGridPoint*** targetPoints, int n, int m);
	HullGridPoints::HullGridPoint*** findXZHullPoints(Data::Part* part, int n, int m);
	HullGridPoints::HullGridPoint*** findXYHullPoints(Data::Part* part, int n, int m);
	HullGridPoints::HullGridPoint*** findYZHullPoints(Data::Part* part, int n, int m);

public:
	class HullGridPoint {
	public:
		Data::Vertex* vertex;
		float dist;
	public:
		HullGridPoint() {
			vertex = NULL;
			dist = 1000.0;
		}
	};

	class HullVertexPair {
	public:
		Eigen::Vector4f translation;
		Data::Vertex* vertex;
		Data::Vertex* pair;
	};

	class VertexDist {
	public:
		Eigen::Vector4f translation;
		Data::Vertex* vertex;
		float dist;

		bool operator< (const VertexDist& vd) const {
			return dist < vd.dist;
		}

		bool operator==(const VertexDist& vd) const
		{
			return (vertex == vd.vertex);
		}
	};
};
