#include <iostream>
#include "data.h"

bool Data::Vertex::calculateVertexNormal()
{
	float nx = 0.0, ny = 0.0, nz = 0.0;
	int counter = 0;

	this->Q = Eigen::Matrix4f::Zero();
	Data::W_edge *e0 = this->edge;
	Data::W_edge *edge = e0;
	do {
		if (! edge) {
			std::cout << "break" << std::endl;
			return false;
		}
		// std::cout << edge->start->id+1 << " " << edge->end->id+1 << std::endl;
		if (edge->end == this) {
			if (edge->right) {
				nx += edge->right->normal.x();
				ny += edge->right->normal.y();
				nz += edge->right->normal.z();
				this->Q += edge->right->normal * edge->right->normal.transpose();
				counter++;
			}
			edge = edge->right_next;
		} else {
			if (edge->left) {
				nx += edge->left->normal.x();
				ny += edge->left->normal.y();
				nz += edge->left->normal.z();
				this->Q += edge->left->normal * edge->left->normal.transpose();
				counter++;
			}
			edge = edge->left_next;
		}
	} while(edge != e0);

	nx /= counter;
	ny /= counter;
	nz /= counter;

	this->normal[0] = nx;
	this->normal[1] = ny;
	this->normal[2] = nz;

	return true;
}

Data::Vertex* Data::Vertex::getMidPoint(const Data::Vertex* v1, const Data::Vertex* v2) {
	Data::Vertex* m = new Data::Vertex;
	m->Q = v1->Q + v2->Q;

	Eigen::Matrix4f tmp = m->Q;
	tmp(3, 0) = 0.;
	tmp(3, 1) = 0.;
	tmp(3, 2) = 0.;
	tmp(3, 3) = 1.;

	Eigen::Vector4f I(0, 0, 0, 1);
	if (tmp.determinant() > 1e-7) {
		m->pos = tmp.inverse() * I;
	} else {
		m->pos = (v1->pos + v2->pos) / 2.0;
	}
	m->pos /= m->pos[3]; // TODO: check
	m->edge = v1->edge;
//	m->pos[3] = 1;

	return m;
}
