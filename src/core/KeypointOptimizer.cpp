#include "KeypointOptimizer.h"
#include "utils/matrixutil.h"

KeypointOptimizerSingleDof::KeypointOptimizerSingleDof(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex) : p1(p1_ex), p2(p2_ex)
{
}

template <typename T>
bool KeypointOptimizerSingleDof::operator()(const T *const x2, T *residuals) const
{
    //return Ta * a - Tb * b.
    const Eigen::Matrix<T, 3, 1> tmp = p1.head<3>().cast<T>() - getTOpt(x2) * p2;
    residuals[0] = tmp(0);
    residuals[1] = tmp(1);
    residuals[2] = tmp(2);
    return true;
}
template bool KeypointOptimizerSingleDof::operator()<ceres::Jet<double, 6>>(const ceres::Jet<double, 6> *const x2, ceres::Jet<double, 6>* residuals) const; 
template bool KeypointOptimizerSingleDof::operator()<double>(const double *const x2, double* residuals) const; 

KeypointOptimizer::KeypointOptimizer(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex) : p1(p1_ex), p2(p2_ex)
{
}

template <typename T>
bool KeypointOptimizer::operator()(const T *const x1, const T *const x2, T *residuals) const
{
    //return Ta * a - Tb * b.
    const Eigen::Matrix<T, 3, 1> tmp = getTOpt(x1) * p1 - getTOpt(x2) * p2;
    residuals[0] = tmp(0);
    residuals[1] = tmp(1);
    residuals[2] = tmp(2);
    return true;
}
template bool KeypointOptimizer::operator()<ceres::Jet<double, 6>>(const ceres::Jet<double, 6> *const x1, const ceres::Jet<double, 6> *const x2, ceres::Jet<double, 6>* residuals) const; 
template bool KeypointOptimizer::operator()<ceres::Jet<double, 12>>(const ceres::Jet<double, 12> *const x1, const ceres::Jet<double, 12> *const x2, ceres::Jet<double, 12>* residuals) const; 
template bool KeypointOptimizer::operator()<double>(const double *const x1, const double *const x2, double* residuals) const; 

KeypointOptimizerNumeric::KeypointOptimizerNumeric(const Eigen::Vector4d &p1_ex, const Eigen::Vector4d &p2_ex) : p1(p1_ex), p2(p2_ex) 
{
}

bool KeypointOptimizerNumeric::operator()(const double *const x1, const double *const x2, double *residuals) const
{
    //return Ta * a - Tb * b.
    const Eigen::Vector4d tmp = getT(x1) * p1 - getT(x2) * p2; //todo dont need fourht row
    residuals[0] = tmp(0);
    residuals[1] = tmp(1);
    residuals[2] = tmp(2);
    return true;
}

