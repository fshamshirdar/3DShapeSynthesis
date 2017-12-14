#pragma once

#include "data.h"
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

// typedef std::pair<Eigen::Matrix3f, Eigen::Vector3f> TransformType;
// typedef std::vector<Eigen::Vector3f>                PointsType;

class Utils {
public:
	static std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransformWithoutScale(const std::vector<Eigen::Vector3f>& src, const std::vector<Eigen::Vector3f>& dst);
	static std::pair<Eigen::Matrix3f, Eigen::Vector3f> computeRigidTransform(const std::vector<Eigen::Vector3f>& src, const std::vector<Eigen::Vector3f>& dst);
	std::string generateRandomString(size_t length);
};
