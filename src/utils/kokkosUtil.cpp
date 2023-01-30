#include "kokkosUtil.h"

Kokkos::View<float[4][4]> eigentoView(const Eigen::Matrix4f &M_eig)
{
    Kokkos::View<float[4][4]> out("Eigen to View conversion matrix");
    auto V_h = create_mirror_view(out);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            V_h(i, j) = M_eig(i, j);
        }
    }
    deep_copy(out, V_h);
    return out;
}
//result is in default execution space
Kokkos::View<Kmat4f> EigentoKmat4f(const Eigen::Matrix4f &T)
{
    // Kokkos::View<float[4][4]> tmp("Eigen to View conversion matrix");
    Kokkos::View<Kmat4f> out("Eigen to View conversion matrix");
    auto V_h = create_mirror_view(out);
    V_h().r1.x = T(0, 0);
    V_h().r1.y = T(0, 1);
    V_h().r1.z = T(0, 2);
    V_h().r1.w = T(0, 3);

    V_h().r2.x = T(1, 0);
    V_h().r2.y = T(1, 1);
    V_h().r2.z = T(1, 2);
    V_h().r2.w = T(1, 3);

    V_h().r3.x = T(2, 0);
    V_h().r3.y = T(2, 1);
    V_h().r3.z = T(2, 2);
    V_h().r3.w = T(2, 3);

    V_h().r4.x = T(3, 0);
    V_h().r4.y = T(3, 1);
    V_h().r4.z = T(3, 2);
    V_h().r4.w = T(3, 3);
    deep_copy(out, V_h);
    return out;
}

Kokkos::View<Kvec4f> EigentoKvec4f(const Eigen::Vector4f &T)
{
    Kokkos::View<Kvec4f> out("Eigen to View conversion vector");
    auto V_h = create_mirror_view(out);
    V_h().x = T(0);
    V_h().y = T(1);
    V_h().z = T(2);
    V_h().w = T(3);
    deep_copy(out, V_h);
    return out;
}

// KOKKOS_FUNCTION Kvec3f transformPoint(const Kvec3f &p, const Kokkos::View<float[4][4]> E_)
// {
//     Kvec3f out;
//     out.x = p.x * E_(0, 0) + p.y * E_(0, 1) + p.z * E_(0, 2) + E_(0, 3);
//     out.y = p.x * E_(1, 0) + p.y * E_(1, 1) + p.z * E_(1, 2) + E_(1, 3);
//     out.z = p.x * E_(2, 0) + p.y * E_(2, 1) + p.z * E_(2, 2) + E_(2, 3);
//     return out;
// }

KOKKOS_FUNCTION Kvec3f transformNormal(const Kvec3f &p, const Kokkos::View<float[4][4]> E_)
{
    Kvec3f out;
    out.x = p.x * E_(0, 0) + p.y * E_(0, 1) + p.z * E_(0, 2);
    out.y = p.x * E_(1, 0) + p.y * E_(1, 1) + p.z * E_(1, 2);
    out.z = p.x * E_(2, 0) + p.y * E_(2, 1) + p.z * E_(2, 2);
    return out;
}


template <typename SpaceType>
void transformThisPCD(SpaceType &space, Kokkos::View<Kvec3f **> pcd, Kokkos::View<Kmat4f> T){
    using namespace Kokkos;

    parallel_for(
        "PCD Transform Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {pcd.extent(0), pcd.extent(1)}), KOKKOS_LAMBDA(int i, int j) {
            pcd(i,j) = T() * pcd(i,j);
        });
}

template void transformThisPCD<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<Kvec3f **> pcd, const Kokkos::View<Kmat4f> T);

template <typename SpaceType>
Kokkos::View<Kvec3f **> transformPCD(SpaceType &space, Kokkos::View<Kvec3f **> pcd, Kokkos::View<Kmat4f> T)
{
    using namespace Kokkos;

    Kokkos::View<Kvec3f **> out("PCD Transform", pcd.extent(0), pcd.extent(1));

    parallel_for(
        "PCD Transform Kernel 2", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {pcd.extent(0), pcd.extent(1)}), KOKKOS_LAMBDA(int i, int j) {
            out(i, j) = T() * pcd(i, j);
        });
    return out;
}

template Kokkos::View<Kvec3f **> transformPCD<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<Kvec3f **> pcd, const Kokkos::View<Kmat4f> T);
