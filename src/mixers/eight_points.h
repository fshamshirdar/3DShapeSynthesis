#pragma once

#include "data.h"
#include "mix_match.h"

class EightPoints: public MixMatch {
public:
	Data* mix(Data* chair1, Data* chair2);
	Data::Vertex** find8Points(Data::Part* part);

public:
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
