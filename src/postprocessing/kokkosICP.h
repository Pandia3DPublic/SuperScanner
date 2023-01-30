 #pragma once
#include "core/Frame.h"
#include "core/Chunk.h"
enum ICPTypes
{
    PointToPoint,
    PointToPlane,
};

//for calling with Frame
template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, std::shared_ptr<Frame> f1, std::shared_ptr<Frame> f2, ICPTypes ICPType = PointToPoint, float * rsme = nullptr);

//for calling with Chunk
template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, std::shared_ptr<Chunk> c1, std::shared_ptr<Chunk> c2, ICPTypes ICPType = PointToPoint, float *rsme= nullptr);

//for calling with depth views
template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, Kokkos::View<unsigned short **> p1, Kokkos::View<unsigned short **> p2,
                          const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2, ICPTypes ICPType = PointToPoint, float *rsme_extern = nullptr);