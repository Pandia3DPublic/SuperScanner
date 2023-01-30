#pragma once
#include "core/Model.h"

std::shared_ptr<open3d::geometry::TriangleMesh> ColorOptMesh(Model &m);
template <typename SpaceType>
void reintegratePP(SpaceType &space, Model &m);
//postprocessing thread
void PostProcessingThreadFunction(Model &m);
