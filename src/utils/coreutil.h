#pragma once
#include "core/Frame.h"
#include "core/Chunk.h"
#include "core/Model.h"
#include "frustum.h"
#include "GlobalDefines.h"

//these methods assume something about the coordinate system
Eigen::Vector2i pointToImgCoords(const Eigen::Vector3d &p, const Eigen::Matrix3d &intr);
Eigen::Vector3d imgCoordstoPoint(const Eigen::Vector2i &c, float &d, const Eigen::Matrix3d &intr);
void prepareDatapath(std::string& s);
open3d::camera::PinholeCameraIntrinsic getLowIntr(open3d::camera::PinholeCameraIntrinsic intrinsic);
open3d::camera::PinholeCameraIntrinsic getScaledIntr(open3d::camera::PinholeCameraIntrinsic intrinsic, int width, int height);
//own data from file, not production relevant
bool checkValid(Model& c);
bool checkValid(const std::shared_ptr<Chunk> c, std::vector<int> &removeIndices);
bool checkValidModel(Model &m, std::vector<int> &removeIndices);
bool checkValidStrict(Model& m, std::vector<std::shared_ptr<Frame>>& framesToRemove);
bool getBit(const unsigned char& a, const int& n);
void setBitOne(unsigned char& a, const int& n);

//get the initial 6 dofs for optimization for a intrachunk
std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> getDoffromKabschChunk(const std::shared_ptr<Chunk> c);
//get the initial 6 dofs for optimization for model opt
std::vector<Eigen::Vector6d,Eigen::aligned_allocator<Eigen::Vector6d>> getInitialDofs(Model &m);

template <typename SpaceType>
void generateFrame(Kokkos::View<unsigned char **[3]> rgb, Kokkos::View<unsigned short **> depth, std::shared_ptr<Frame> out, SpaceType& space);
std::shared_ptr<Frame> getSingleFrame(std::list<std::shared_ptr<Frame>> &recordbuffer);

void setDefaultIntrinsic();
int getdircount(std::string path);
std::string getPicNumberString(int a);
void setFromIntrinsicFile(const std::string &filepath);
bool fileExists(const std::string& filename);

#ifdef RECORDBUFFER
void saveImagestoDisc(const std::string& path, std::list <std::shared_ptr<Frame>>& recordbuffer);
#endif

#ifdef SAVETRAJECTORY
void saveTrajectorytoDisk(const std::string& path, Model& m, std::string name = "trajectory.txt");
void saveChunkTrajectorytoDisk(const std::string &path, Model &m, std::string name = "chunk_trajectory.txt");
#endif
void readTrajectory(const std::string &path, std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses);
