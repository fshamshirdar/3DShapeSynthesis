#pragma once

#include "data.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

typedef std::pair<Eigen::Matrix3f, Eigen::Vector3f> TransformType;
typedef std::vector<Eigen::Vector3f>                PointsType;

class Utils {
	static TransformType computeRigidTransform(const PointsType& src, const PointsType& dst);
};
