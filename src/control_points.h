#pragma once

#include <string>
#include "data.h"

class ControlPointsMiner {
public:
	/**
	Class ControlPoint
	*/
	class ControlPoint {
	public:
		Eigen::Vector4f translation;
		Data::Vertex* vertex;
		Data::Vertex* pair;
		float dist;
	};

	/**
	Class VertexControlPoint
	*/
	class VertexControlPoint {
	public:
		Eigen::Vector4f translation;
		Data::Vertex* vertex;
		float dist;

		bool operator< (const VertexControlPoint& vd) const
		{
			return dist < vd.dist;
		}

		bool operator==(const VertexControlPoint& vd) const
		{
			return (vertex == vd.vertex);
		}
	};

public:
	virtual std::vector<ControlPointsMiner::ControlPoint*> findControlPoints(Data::Part* ref, Data::Part* target) = 0;
};
