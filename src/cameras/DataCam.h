#pragma once
#include "core/Frame.h"

template <typename SpaceType>
std::shared_ptr<Frame> getSingleFrame(std::string path, int nstart, SpaceType& space);

void DataCamThreadFunction(std::atomic<bool> &stop);