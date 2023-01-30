#pragma once

#include "Frame.h"
#include "Optimizable.h"
#include "frustum.h"
#include "c_keypoint.h"


//bool matchsort(match &i, match &j);

class Chunk : public Optimizable, public KeypointUnit{
public:

	Chunk();
	~Chunk();
	//base variables
	std::vector<std::shared_ptr<Frame>> frames;

	//std::vector<c_keypoint> keypoints;
	int ncorframes; // number edges between frames
	bool chunktransapplied = false;
	std::shared_ptr<Frustum> frustum; // only gets set after chunk is complete
	Eigen::Vector3d imuPosition = Eigen::Vector3d::Zero(); //used to save the imu positon for the imu filter

	//methods
	//void performSparseOptimization();
	bool OptimizeAlignment(const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &initx, bool recursiveCall = false); //the new shit
	void generateChunkKeypoints(); //keypoints used for global alignment, fuse same keypoints
	void generateEfficientKeypoints(); //keypoints used for sparse optimization
	void deleteStuff(); //delete everything that is not needed when chunk is finished
	bool doFrametoModelforNewestFrame();
	void generateFrustum();
	void setWorldTransforms(); //uses chunktoworldtrans and chunktransform to set frametoworldtransform

	//for visualization
	//variables
	std::shared_ptr<open3d::geometry::LineSet> ls = std::make_shared<open3d::geometry::LineSet>();
	std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> spheres;
	//method
	void markChunkKeypoints();

	//debugging
	bool output = true;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
	void getMajorityDescriptor(const std::vector<int>& indeces,const std::vector<c_keypoint>& kps, std::vector<uchar>& des);
};

