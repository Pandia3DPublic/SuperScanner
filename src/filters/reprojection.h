#include "match.h"
#include "core/Frame.h"

void reprojectionfilterO3DPcd(std::shared_ptr<Frame> f1, std::shared_ptr<Frame> f2, pairTransform &trans, std::vector<match> &matches);
template <typename SpaceType>
void reprojectionfilter(SpaceType &space, pairTransform &trans);
