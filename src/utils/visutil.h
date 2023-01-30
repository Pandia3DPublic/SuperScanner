#pragma once

#include "core/Frame.h"
#include "core/Chunk.h"
#include "core/Model.h"
#include "imageutil.h"
#include "c_keypoint.h"
#include "VoxelGrid/kokkosFunctions.h"

std::shared_ptr<open3d::geometry::TriangleMesh> getCameraPathMesh(Model& m);
std::shared_ptr<open3d::geometry::TriangleMesh> createPathMesh(Model &m);

void visualizeChunk(Chunk& c, bool worldcoords = false);
void visualizetsdfModel(Model& m);
//other data formats
void drawEigen(std::vector<Eigen::Matrix3Xd> vvec); //draws an eigen matrix containing points
void drawPointVector(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> v); //draws a vector containing eigen 3d poitns
void visualizeHomogeniousVector(std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &v);
//rest
void visualizecustomskeypoints(std::vector<c_keypoint>& kps);
//draws all pcds with keypoints and lines between them
void visualizecurrentMatches(std::shared_ptr<Chunk> c);
//draws all pcds with keypoints and lines between them
void visualizecurrentMatches(Model& m, bool raw = false);
void visualizeNearestPoints(Kokkos::View<Kvec3f **> x_nearest, Kokkos::View<Kvec3f **> x, Kokkos::View<Kvec3f **> y, int sampling = 1);

std::shared_ptr<open3d::geometry::LineSet> getCamera(Eigen::Matrix4d& t, Eigen::Vector3d color = Eigen::Vector3d(0.0,0.0,1.0));
std::shared_ptr<open3d::geometry::LineSet> getCameraPath(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v, Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0));

std::shared_ptr<open3d::geometry::LineSet> getOrigin();
std::shared_ptr<open3d::geometry::LineSet> getvisFrusti(Model& m, Frustum& corefrustum);
std::shared_ptr<open3d::geometry::LineSet> getFrustumLineSet(Frustum& fr,const Eigen::Vector3d& color);

std::shared_ptr<open3d::geometry::TriangleMesh> getCameraPathMesh(Model& m, int& divider);

template <typename T>
std::shared_ptr<open3d::geometry::PointCloud> ViewtoPcd(T pcd)
{
    auto pcd_h = Kokkos::create_mirror_view(pcd);
    Kokkos::deep_copy(pcd_h, pcd);
    std::shared_ptr<open3d::geometry::PointCloud> out;
    out = std::make_shared<open3d::geometry::PointCloud>();

    for (int i = 0; i < pcd_h.extent(0); i++)
    {
        for (int j = 0; j < pcd_h.extent(1); j++)
        {
            Eigen::Vector3d tmp(pcd_h(i, j).x, pcd_h(i, j).y, pcd_h(i, j).z);
            out->points_.push_back(tmp);
        }
    }

    return out;
}
