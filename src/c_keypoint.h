#pragma once

//keypoint with descriptor
struct c_keypoint
{
	Eigen::Vector4d p;
	std::vector<unsigned char> des;
	// overload equal operator
	bool operator==(const c_keypoint &k);
	void transform(const Eigen::Matrix4d &m);
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};