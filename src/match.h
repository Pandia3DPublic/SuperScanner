#pragma once
#include "core/Frame.h"
class Chunk; // forward declaration to break cycle
// #include "core/Chunk.h"
// #include "core/Optimizable.h"
// #include "KeypointUnit.h"
// #include "GlobalDefines.h"
// #include "VoxelGrid/kokkosFunctions.h"

//for rawmatches and matches (frame pairs). Indeces fit to keypoints variable, not validkeypoints
//p1 always refers to the frame with lower unique id.
struct match
{
	double d;
	Eigen::Vector2i indeces;
	Eigen::Vector4d p1; //lives in the pcd of the pointer k1 in pairTransform
	Eigen::Vector4d p2; //lives in the pcd of the pointer k2 in pairTransform
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	friend std::ostream &operator<<(std::ostream &os, const match &r);
};


//contains the kabschtrans for one constraint and the ideal transforms
struct pairTransform
{
	pairTransform(std::shared_ptr<KeypointUnit> a_ex, std::shared_ptr<KeypointUnit> b_ex): k1(a_ex),k2(b_ex){}
	pairTransform(){}

	bool set = false;
	std::vector<match> filteredmatches;
	std::shared_ptr<KeypointUnit> k1;
	std::shared_ptr<KeypointUnit> k2;
	bool kabschIsAtoB = true; //controls which transformation is returned by getTransformationFrom

	Eigen::Matrix4d getTransformationFrom(std::shared_ptr<KeypointUnit> a, std::shared_ptr<KeypointUnit> b);
	
	
	void setkabschtrans(const Eigen::Matrix4d& kt){kabschtrans =kt;}
	void setinvkabschtrans(const Eigen::Matrix4d &ikt){invkabschtrans = ikt;}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	private:
	Eigen::Matrix4d kabschtrans = Eigen::Matrix4d::Identity();	  //transform from k1 to k2
	Eigen::Matrix4d invkabschtrans = Eigen::Matrix4d::Identity(); //transform from k2 to k1

};

struct pair_hash
{
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2> &p) const
    {

		auto hash1 = std::hash<T1>{}(p.first);
		auto hash2 = std::hash<T2>{}(p.second);
		return hash1 ^ hash2;
		// return std::hash<T1>{}(pair.first) ^ std::hash<T2>{}(pair.second);
    }
};

class c_umap
{
private: 
	pairTransform emptyTransform; //empty pairTransform that is returned if an element does not exist in x
public:
	// std::unordered_map<std::pair<std::shared_ptr<Frame>, std::shared_ptr<Frame>>, pairTransform, frame_pair_hash> x;
	std::unordered_map<std::pair<std::shared_ptr<KeypointUnit>, std::shared_ptr<KeypointUnit>>, pairTransform, pair_hash> x;
	pairTransform &operator()(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2);
	pairTransform &addElement(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2);
	void removeElement(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2);
};



struct unique_id_counter
{
	int id = 0;

	operator int()
	{
		id++;
		return id - 1;
		;
	}
};
