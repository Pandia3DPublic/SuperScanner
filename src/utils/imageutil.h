#pragma once
#include "VoxelGrid/kokkosFunctions.h"

//this is all on gpu
template <typename SpaceType>
Kokkos::View<unsigned char **[3]> ResizeColorView(SpaceType &space, Kokkos::View<unsigned char **[3]> rgb, int resi, int resj, bool bilinear = true);
template <typename SpaceType>
Kokkos::View<unsigned short **> ResizeDepthView(SpaceType &space, Kokkos::View<unsigned short **> depth, int resi, int resj);
template <typename SpaceType>
Kokkos::View<Kvec3f **> CreatePcdFromDepthView(SpaceType &space, Kokkos::View<unsigned short **> depth, const open3d::camera::PinholeCameraIntrinsic &intr);
template <typename SpaceType>
Kokkos::View<Kvec3f **> CalculateNormalsFromPCDView(SpaceType &space, Kokkos::View<Kvec3f **> pcd);
template <typename SpaceType>
Kokkos::View<float **> CalculateIntensityfromColorView(SpaceType &space, Kokkos::View<unsigned char **[3]> rgb);
template <typename SpaceType>
Kokkos::View<float **> NNSBruteForce(SpaceType &space, Kokkos::View<Kvec3f **> pcd_source, Kokkos::View<Kvec3f **> pcd_target);

void topencv(std::shared_ptr<open3d::geometry::Image> &img, cv::Mat &cvimg);
void topen3d(cv::Mat &cvimg, std::shared_ptr<open3d::geometry::Image> &img);

//for vis
std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<unsigned char **[3]> img);
std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<Kvec3f **> img);
std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<unsigned short **> img);
std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<float **> img);
std::shared_ptr<open3d::geometry::PointCloud> ViewtoO3dPCd(Kokkos::View<Kvec3f **> img);
void AddViewNormalstoO3DPCD(Kokkos::View<Kvec3f **> normals, std::shared_ptr<open3d::geometry::PointCloud> pcd);



//depricated
std::shared_ptr<open3d::geometry::Image> resizeImage(std::shared_ptr<open3d::geometry::Image> image_ptr, int width, int height, std::string mode);
template <typename SpaceType>
std::shared_ptr<open3d::geometry::Image> ColorViewtoImageDownscale(Kokkos::View<unsigned char **[3]> color, int resj, int resi, std::string mode, SpaceType &space);
template <typename SpaceType>
std::shared_ptr<open3d::geometry::Image> DepthViewtoImageDownscale(Kokkos::View<unsigned short **> depth, int resj, int resi, std::string mode, SpaceType &space);
