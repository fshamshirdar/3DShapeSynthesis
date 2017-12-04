#ifndef __WEDS_H__
#define __WEDS_H__

#include <string>
#include <Eigen/Dense>
#include <GL/glui.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

class WingedEdge {
public:
	class Vertex;
	class W_edge;
	class Face;

	class Region {
	public:
		int		id;
		std::string	name;
		std::vector<WingedEdge::Vertex*> vertices; // dynamic array
		std::vector<WingedEdge::Face*> faces;  // dynamic array
		std::vector<WingedEdge::W_edge*> edges;

		std::vector<WingedEdge::Region*> neighbors;
	};

	class W_edge {
	public:
		Vertex	*start, *end;
		Face	*left, *right;
		W_edge	*left_prev, *left_next;
		W_edge	*right_prev, *right_next;
	};

	class Vertex {
	public:
		int	id;
		int	regionId;
		Eigen::Vector4f	pos;
		W_edge	*edge;
		Eigen::Vector4f	normal; // smooth shaded -> all the faces of the vertices and avg normals
//		std::vector<Vertex*> pairs;
		Eigen::Matrix4f	Q;
	};

	class Face {
	public:
		int	id;
		int	regionId;
		W_edge	*edge;
		Vertex	*v1, *v2, *v3;
		Eigen::Vector4f normal;
		// GLfloat	nx, ny, nz, nw; // flat shaded -> normal of plane
	};

	class SimplificationPair {
	public:
		bool operator< (const SimplificationPair& sp) const {
			return cost < sp.cost;
		}

		bool operator==(const SimplificationPair& other) const
		{
			return (v1 == other.v1 && v2 == other.v2);
		}

	public:
		Vertex	*v1, *v2, *mid;
		float	cost;
	};
};

#endif
