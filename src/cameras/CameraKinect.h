#pragma once
#include "core/Frame.h"


void KinectThread(std::atomic<bool>& stop, std::atomic<bool>& cameraParameterSet);
