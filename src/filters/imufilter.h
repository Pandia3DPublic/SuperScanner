#pragma once
#include "core/Chunk.h"

//c1 is old chunk
//c2 is new chunk with imu pos
// void imufilter(shared_ptr<Chunk> c1, shared_ptr<Chunk> c2, pairTransform &trans, std::vector<match> &matches);
//@df last frame of last chunk before currentChunk
void jumpingFilter(pairTransform &trans, std::shared_ptr<Frame> df);
