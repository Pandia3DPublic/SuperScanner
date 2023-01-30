#pragma once
// #include "Optimizable.h"
#include "KeypointUnit.h"
#include "GlobalDefines.h"
#include "VoxelGrid/kokkosFunctions.h"


struct indexPointPair
{
	Eigen::Vector3d p;
	int index;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class Frame : public KeypointUnit{
public:
    Frame();
    ~Frame();

	//basic variables
	//these get allocated and filled on the gpu initialy
	Kokkos::View<unsigned char **[3]> rgb; //do not allocate. But use shallow copy to init these in generateFrame
	Kokkos::View<unsigned short **> depth;
	// std::shared_ptr<open3d::geometry::PointCloud> lowpcd;
	Kokkos::View<Kvec3f **, Kokkos::LayoutRight, Kokkos::HostSpace> lowpcd_h; //for generateFrustum
	Kokkos::View<Kvec3f **> lowpcd;
	Kokkos::View<Kvec3f **> lowpcd_normals;
	Kokkos::View<float **> lowpcd_intensity;
	// Kokkos::View<unsigned char **[3]> lowpcd_rgb;
	std::shared_ptr<std::vector<k4a_imu_sample_t>> imuVector; //vector contains imu samples since last frame
	unsigned long timestamp; //timestamp of frame data in microseconds.

	//used for gpu-cpu streaming
	Kokkos::View<unsigned char[G_RESI][G_RESJ][3], Kokkos::LayoutLeft, Kokkos::HostSpace> rgb_h;
	Kokkos::View<unsigned short[G_RESI][G_RESJ], Kokkos::LayoutLeft, Kokkos::HostSpace> depth_h;
	void MovetoCPU(); //as long as its only these two accessing the views, we dont need locks
	void MovetoGPU(); //as long as its only these two accessing the views, we dont need locks
	bool dataOnGPU{true};
	static std::atomic<int> nGPUFrames;
	std::mutex viewLock; //should be unecessary
	static Kokkos::View<unsigned char **[3]> rgbDeallocHelper;
	static Kokkos::View<unsigned short **> depthDeallocHelper;

	bool integrated = false; //this is used in multiple threads
	bool worldtransset = false;
	bool duplicate = false; // to avoid double integration
	bool pushedIntoIntegrationBuffer = false;
	Eigen::Vector6d integrateddofs = Eigen::Vector6d::Zero();
	Eigen::Matrix4d chunktransform = Eigen::Matrix4d::Identity(); //within the chunk


	std::string rgbPath;
	std::string depthPath;
	
	Eigen::Vector6d getWorlddofs();
	Eigen::Matrix4d getFrametoWorldTrans();
	void setFrametoWorldTrans(const Eigen::Matrix4d& a);
	void setworlddofs(const Eigen::Vector6d& a);

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	//debug
	// int integratecounter =0;
	// int vbcounter =0;
	// Eigen::Matrix4f integratematrix = Eigen::Matrix4f::Zero();
private:
	Eigen::Matrix4d frametoworldtrans= Eigen::Matrix4d::Identity(); //world coordiante transformation. Is garanteed to be equal to worlddofs. Must be handled threadsafe with integrationlock
	Eigen::Vector6d worlddofs = Eigen::Vector6d::Zero(); //contains the dofs of the most current world trans. Is garanteed to be equal to frametoworldtrans. First rot than trans
	//getter and setter taht are actually important!




};

