#pragma once

//accf2 automatic derivatives
//functors for ceres non linear opt autodiff
struct KeypointOptimizerSingleDof
{
	KeypointOptimizerSingleDof(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex);

	template <typename T>
	bool operator()(const T *const x2, T *residuals) const;

	const Eigen::Vector4d p1;
	const Eigen::Vector4d p2;
};

struct KeypointOptimizer
{
	KeypointOptimizer(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex);

	template <typename T>
	bool operator()(const T *const x1, const T *const x2, T *residuals) const;

	const Eigen::Vector4d p1;
	const Eigen::Vector4d p2;
};

struct KeypointOptimizerNumeric
{
	KeypointOptimizerNumeric(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex);

	bool operator()(const double *const x1, const double *const x2, double *residuals) const;

	const Eigen::Vector4d p1;
	const Eigen::Vector4d p2;
};
