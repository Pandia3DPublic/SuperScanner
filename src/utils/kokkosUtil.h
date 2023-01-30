#pragma once
#include "VoxelGrid/kokkosFunctions.h"
Kokkos::View<float[4][4]> eigentoView(const Eigen::Matrix4f &M_eig);
Kokkos::View<Kmat4f> EigentoKmat4f(const Eigen::Matrix4f &T);
Kokkos::View<Kvec4f> EigentoKvec4f(const Eigen::Vector4f &T);
// KOKKOS_FUNCTION Kvec3f transformPoint(const Kvec3f &p, const Kokkos::View<float[4][4]> E_);
KOKKOS_FUNCTION Kvec3f transformNormal(const Kvec3f &p, const Kokkos::View<float[4][4]> E_);

template <typename SpaceType>
void transformThisPCD(SpaceType &space, Kokkos::View<Kvec3f **> pcd, Kokkos::View<Kmat4f> T);
template <typename SpaceType>
Kokkos::View<Kvec3f **> transformPCD(SpaceType &space, Kokkos::View<Kvec3f **> pcd, Kokkos::View<Kmat4f> T);