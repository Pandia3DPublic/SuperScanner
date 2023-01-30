#pragma once

#include "Chunk.h"
#include "VoxelGrid/GPUVoxelGrid.h"

class Model : public Optimizable
{
public:
	Model();
	~Model();

	std::vector<std::shared_ptr<Chunk>> chunks;
	std::shared_ptr<GPUVoxelGrid> globalGrid;

	std::vector<std::shared_ptr<Chunk>> invalidChunks;
	Eigen::Matrix4d currentPos = Eigen::Matrix4d::Identity(); //current camera position. Only use in integrationlock
	std::list<std::shared_ptr<Frame>> recordbuffer;			  //never used if RECORDBUFFER is not defined

	bool doChunktoModelforNewestChunk();
	void OptimizeAlignment(const std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &initx); //the new shit

	void setWorldTransforms();

	//vector<shared_ptr<Frame>> frames; //contains all frames of the model

	void saveCPUMesh(std::string name);

	void StreamNonLGtoCPU(std::vector<std::shared_ptr<Chunk>> &localGroup);


private:
	bool allChunkshavePos();
};
