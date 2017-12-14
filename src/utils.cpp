#include "utils.h"
#include <iostream>

std::pair<Eigen::Matrix3f, Eigen::Vector3f> Utils::computeRigidTransformWithoutScale(const std::vector<Eigen::Vector3f>& src, const std::vector<Eigen::Vector3f>& dst)
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

std::pair<Eigen::Matrix3f, Eigen::Vector3f> Utils::computeRigidTransform(const std::vector<Eigen::Vector3f>& src, const std::vector<Eigen::Vector3f>& dst)
{
        assert(src.size() == dst.size());
        const size_t n = src.size();

        Eigen::Vector3f v_center_tar3 = Eigen::Vector3f::Zero(), v_center3 = Eigen::Vector3f::Zero();
        for(size_t i = 0; i < n; ++ i) {
                v_center_tar3 += dst[i];
                v_center3 += src[i];
        }
        v_center_tar3 /= double(n);
        v_center3 /= double(n);
        // calculate centers of positions, potentially extend to 3D

        double f_sd2_tar = 0, f_sd2 = 0; // only one of those is really needed
        Eigen::Matrix3f t_cov = Eigen::Matrix3f::Zero();
        for(size_t i = 0; i < n; ++ i) {
                Eigen::Vector3f v_vert_i_tar = dst[i] - v_center_tar3;
                Eigen::Vector3f v_vert_i = src[i] - v_center3;
                // get both vertices

                f_sd2 += v_vert_i.squaredNorm();
                f_sd2_tar += v_vert_i_tar.squaredNorm();
                // accumulate squared standard deviation (only one of those is really needed)

                t_cov.noalias() += v_vert_i * v_vert_i_tar.transpose();
                // accumulate covariance
        }
        // calculate the covariance matrix

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(t_cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // calculate the SVD

        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();
        // compute the rotation

        double f_det = R.determinant();
        Eigen::Vector3f e(1, 1, (f_det < 0)? -1 : 1);
        // calculate determinant of V*U^T to disambiguate rotation sign

        if(f_det < 0)
                R.noalias() = svd.matrixV() * e.asDiagonal() * svd.matrixU().transpose();
        // recompute the rotation part if the determinant was negative

        R = Eigen::Quaternionf(R).normalized().toRotationMatrix();
        // renormalize the rotation (not needed but gives slightly more orthogonal transformations)

        double f_scale = svd.singularValues().dot(e) / f_sd2_tar; 
        double f_inv_scale = svd.singularValues().dot(e) / f_sd2; // only one of those is needed
        // calculate the scale

        R *= f_inv_scale;
        // apply scale

        Eigen::Vector3f t = v_center_tar3 - (R * v_center3); // R needs to contain scale here, otherwise the translation is wrong
        // want to align center with ground truth

        return std::make_pair(R, t); // or put it in a single 4x4 matrix if you like
}

std::string Utils::generateRandomString( size_t length )
{
    return std::string();
}
