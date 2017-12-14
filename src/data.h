#pragma once

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
#include "utils.h"

class Data {
public:
	class Vertex;
	class W_edge;
	class Face;
	class Region;
	class Part;
	class PartIntersection;

	/**
	Class Part
	*/
	class Part {
	public:
		enum Type {
			BACK_SHEET = 1,
			SEAT_SHEET = 2,
			LEFT_HANDLE = 3,
			RIGHT_HANDLE = 4,
			LEG_FRONT_SPINDLE = 5,
			LEG_BASE = 6,
			LEG_BAR = 7,
			LEG_BRANCH = 8,
			LEG_FRONT_LEG = 9,
			LEG_BACK_LEG = 10,
			LEG_BACK_SPINDLE = 11,
			LEG_LEFT_SPINDLE = 12,
			LEG_RIGHT_SPINDLE = 13,

			LEG = 14, // dummy

			FOUR_LEGGED = 20,
			SINGLE_LEGGED = 21,
			TWO_LEGGED = 22,
		};

	public:
		Data*					parent;
		Part::Type				type;
		Eigen::AlignedBox3f			boundingBox;
		std::vector<Data::Region*>		regions;
		std::vector<Data::PartIntersection*>	neighbors;
	public:
		Part();
		bool isChild(Data::Part* parent);
		bool isParent(Data::Part* child);
		void findNeighborsByBoxIntersection();
		void findNeighborsByVertexToFaceDistance();
		void scale(Eigen::Vector3f scale, Eigen::Vector3f base);
		void scale(Eigen::Vector3f scale);
		void scale(Eigen::AlignedBox3f box, Eigen::Vector3f base);
		void scale(Eigen::AlignedBox3f box);
		void translate(Eigen::Vector3f translate);
		void transform(std::pair<Eigen::Matrix3f, Eigen::Vector3f> transformation);
		void resetBoundingBox();
		void recalculateBoundingBox(Data::Vertex* vertex);
		void addVertexToPartIntersection(Data::Part* part, Data::Vertex* vertex, bool mine=true);
		void recalculateNormals();
	};

	/**
	Class PartIntersection
	*/
	class PartIntersection {
	public:
		Eigen::AlignedBox3f		boundingBox;
		Data::Part*			neighbor;
		std::vector<Data::Vertex*>	vertices;
		std::vector<Data::Vertex*>	myVertices;
		std::vector<Data::Vertex*>	neighborVertices;
	public:
		PartIntersection();
		void resetBoundingBox();
		void recalculateBoundingBox(Data::Vertex* vertex);
	};

	/**
	Class Region
	*/
	class Region {
	public:
		int				id;
		std::string			name;
		std::vector<Data::Vertex*>	vertices; // dynamic array
		std::vector<Data::Face*>	faces;  // dynamic array
		std::vector<Data::W_edge*>	edges;

		Eigen::AlignedBox3f		boundingBox;

		std::vector<Data::Region*>	neighbors;
	public:
		Region();
		void resetBoundingBox();
		void recalculateBoundingBox(Data::Vertex* vertex);
		void recalculateNormals();
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
	// Parts
	Data* clone();
	void save();
	Data::Part* findPartByType(Part::Type type);
	void addPart(Data::Part* part);
	void addParts(Data* data);
	void replacePartByType(Data::Part* part);
	void deletePartByType(Data::Part::Type type);

	Data::W_edge* findEdge(const Data::Region* region, const Data::Vertex* v1, const Data::Vertex* v2);
	void findPartsNeighborsByVertexToFaceDistance();
	void findRegionsNeighborsByVertexToFaceDistance(Data::Part* part1, Data::Part* part2);
	void findPartsNeighborsByVertexToFaceDistanceForPart(Data::Part* part);
	void findPartsNeighborsByBoxIntersection();
	void findRegionsNeighborsByBoxIntersection(Data::Part* part1, Data::Part* part2);
	void findPartsNeighborsByBoxIntersectionForPart(Data::Part* part);
	bool isPointWithinTriangle(Data::Face* face, Eigen::Vector3f P);
	bool sameSide(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f A, Eigen::Vector3f B);

	/////////////////
	/// Variables ///
	/////////////////
public:
	std::vector<Data::Part*> parts;
	std::string path;

	int vertices_size;
	int faces_size;
};
