#pragma once

//libs
#include <Kokkos_Core.hpp>
#include <Kokkos_UnorderedMap.hpp> //for some reason this has to go before open3dCuda!!!! Try to get rid of open3d cuda
#include <open3d/Open3D.h>
#include "opencv2/core.hpp"
#include "opencv2/features2d/features2d.hpp"
// #include "opencv2/core/cuda.hpp"
// #include "opencv2/cudafeatures2d.hpp"
#include <k4a/k4a.hpp> //Azure Kinect C++ wrapper
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <limits>
#include <algorithm>
#include <ceres/ceres.h>
#include <random>
#include <memory>
#include <list>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map> 
#include <fstream>
#include <filesystem>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "KokkosHostThreadStuff.h"


//integrated libs
#include "Gui/imgui-docking/imgui.h"
#include "Gui/imgui-docking/imgui_impl_glfw.h"
#include "Gui/imgui-docking/imgui_impl_opengl3.h"
#include "Gui/imgui-docking/FileBrowser/ImGuiFileBrowser.h"

//custom code. If we are actively doing something here, remove it during that time!.
// #include "ptimer.h"
// #include "GlobalDefines.h"
// #include "utils/coreutil.h"
// #include "core/threadCom.h"