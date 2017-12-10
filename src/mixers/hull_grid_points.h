#pragma once

#include "data.h"
#include "mix_match.h"
#include <iostream>

class HullGridPoints: public MixMatch {
public:
	Data* mix(Data* chair1, Data* chair2);

public:
	class HullGridPoint {
	public:
		Data::Vertex* front;
		float dist;
	public:
		HullGridPoint() {
			vertex = NULL;
			dist = 1000.0;
		}
	};

	class ClosestVertexPair {
	public:
		Eigen::Vector4f translation;
		Data::Vertex* vertex;
		Data::Vertex* pair;
		float dist;
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
