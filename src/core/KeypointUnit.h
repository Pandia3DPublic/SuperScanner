#pragma once
#include "c_keypoint.h"
class KeypointUnit
{
public:
	KeypointUnit();
	~KeypointUnit();
	std::vector<c_keypoint> orbKeypoints; //contains all keypoints, after basic filtering for depth
	cv::Mat orbDescriptors; // redundant to c_keypoint descriptors for opencv
	std::vector<c_keypoint> efficientKeypoints; // contains only the non filtered keypoints that are used for sparse alignment. Filled by chunk
	//transformations
	//init is important!
	Eigen::Matrix4d chunktoworldtrans = Eigen::Matrix4d::Identity(); //what comes out of chunk optimize
	int unique_id;
	bool isChunk; //indicates wheter this keypointunit is a chunk
	int structureIndex = -1; //indicates which place in the model this chunk takes. If frame it indicates the chunk place
	Eigen::Vector6d x= Eigen::Vector6d::Zero(); // degrees of freedom for optimization


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:


};

