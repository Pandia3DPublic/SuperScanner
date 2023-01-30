#pragma once

#include "core/Model.h"


int reconrun(Model &m, bool integrate);
std::vector<std::shared_ptr<Chunk>> getLimitedLocalGroup(const std::vector<std::shared_ptr<Chunk>> &localGroup, int ngroup, std::shared_ptr<Chunk> include = nullptr);
