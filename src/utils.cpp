#include "utils.h"
#include <iostream>

TransformType Utils::computeRigidTransform(const PointsType& src, const PointsType& dst)
{
	assert(src.size() == dst.size());
	int pairSize = src.size();
	Eigen::Vector3f center_src(0, 0, 0), center_dst(0, 0, 0);
	for (int i=0; i<pairSize; ++i)
	{
		center_src += src[i];
		center_dst += dst[i];
	}
	center_src /= (double)pairSize;
	center_dst /= (double)pairSize;

	Eigen::MatrixXf S(pairSize, 3), D(pairSize, 3);
	for (int i=0; i<pairSize; ++i)
	{
		for (int j=0; j<3; ++j)
			S(i, j) = src[i][j] - center_src[j];
		for (int j=0; j<3; ++j)
			D(i, j) = dst[i][j] - center_dst[j];
	}
	Eigen::MatrixXf Dt = D.transpose();
	Eigen::Matrix3f H = Dt*S;
	Eigen::Matrix3f W, U, V;

	Eigen::JacobiSVD<Eigen::MatrixXf> svd;
	Eigen::MatrixXf H_(3, 3);
	for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) H_(i, j) = H(i, j);
	svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV );
	if (!svd.computeU() || !svd.computeV()) {
		std::cerr << "decomposition error" << std::endl;
		return std::make_pair(Eigen::Matrix3f::Identity(), Eigen::Vector3f::Zero());
	}
	Eigen::Matrix3f Vt = svd.matrixV().transpose();
	Eigen::Matrix3f R = svd.matrixU()*Vt;
	Eigen::Vector3f t = center_dst - R*center_src;	
	
	return std::make_pair(R, t);
}
