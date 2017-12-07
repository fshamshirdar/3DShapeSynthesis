#ifndef __WEDS_H__
#define __WEDS_H__

#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GL/glui.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

class Data {
public:
	class Vertex;
	class W_edge;
	class Face;

	/**
	Class Region
	*/
	class Region {
	public:
		int		id;
		std::string	name;
		std::vector<Data::Vertex*> vertices; // dynamic array
		std::vector<Data::Face*> faces;  // dynamic array
		std::vector<Data::W_edge*> edges;

		Eigen::AlignedBox3f boundingBox;

		std::vector<Data::Region*> neighbors;
	public:
		Region();
	};

	/**
	Class W_edge
	*/
	class W_edge {
	public:
		Vertex	*start, *end;
		Face	*left, *right;
		W_edge	*left_prev, *left_next;
		W_edge	*right_prev, *right_next;
	};

	/**
	Class Vertex
	*/
	class Vertex {
	public:
		int	id;
		int	regionId;
		Eigen::Vector4f	pos;
		W_edge	*edge;
		Eigen::Vector4f	normal; // smooth shaded -> all the faces of the vertices and avg normals
//		std::vector<Vertex*> pairs;
		Eigen::Matrix4f	Q;

	public:
		bool calculateVertexNormal();
		Data::Vertex* getMidPoint(const Data::Vertex* v1, const Data::Vertex* v2);
	};

	/**
	Class Face
	*/
	class Face {
	public:
		int	id;
		int	regionId;
		W_edge	*edge;
		Vertex	*v1, *v2, *v3;
		Eigen::Vector4f normal;
		// GLfloat	nx, ny, nz, nw; // flat shaded -> normal of plane

	public:
		bool calculateFaceNormal();
	};

	/**
	Class SimplificationPair
	*/
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

/////////////////
/// Functions ///
/////////////////
public:
	Data::W_edge* findEdge(const Data::Region* region, const Data::Vertex* v1, const Data::Vertex* v2);
	void findRegionsNeighborsByVertexToFaceDistance();
	bool isPointWithinTriangle(Data::Face* face, Eigen::Vector3f P);
	bool sameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f A, Eigen::Vector3f B);

/////////////////
/// Variables ///
/////////////////
public:
	std::vector<Data::Region*> regions;

	int vertices_size;
	int faces_size;
};

#endif
