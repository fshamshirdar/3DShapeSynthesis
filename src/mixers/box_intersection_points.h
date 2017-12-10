#pragma once

#include "data.h"
#include "mix_match.h"
#include <iostream>

class BoxIntersectionPoints: public MixMatch {
public:
	Data* mix(Data* chair1, Data* chair2);

public:
	class VertexPair {
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
