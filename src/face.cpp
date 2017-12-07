#include "data.h"

bool Data::Face::calculateFaceNormal()
{
	float ux = this->v2->pos.x() - this->v1->pos.x();
	float uy = this->v2->pos.y() - this->v1->pos.y();
	float uz = this->v2->pos.z() - this->v1->pos.z();

	float vx = this->v3->pos.x() - this->v1->pos.x();
	float vy = this->v3->pos.y() - this->v1->pos.y();
	float vz = this->v3->pos.z() - this->v1->pos.z();

	float nx = (uy * vz) - (uz * vy);
	float ny = (uz * vx) - (ux * vz);
	float nz = (ux * vy) - (uy * vx);
	GLfloat length = sqrt(nx*nx + ny*ny + nz*nz);
	if (length < 1e-7) {
		this->normal[0] = this->normal[1] = this->normal[2] = this->normal[3] = 0.0;
		return false;
	} else {
		this->normal[0] = nx / length;
		this->normal[1] = ny / length;
		this->normal[2] = nz / length;
		this->normal[3] =
			- this->normal.x() * this->v1->pos.x()
			- this->normal.y() * this->v1->pos.y()
			- this->normal.z() * this->v1->pos.z();
	}

	return true;
}
