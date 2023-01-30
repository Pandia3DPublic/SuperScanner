#pragma once
Eigen::Matrix3d kabsch(const Eigen::Matrix3Xd &in, const Eigen::Matrix3Xd &out);
Eigen::Vector3d RottoDof(const Eigen::Matrix3d &R);
void TestDOF(const Eigen::Matrix3d& R);