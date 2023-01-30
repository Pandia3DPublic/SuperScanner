#include "c_keypoint.h"

bool c_keypoint::operator==(const c_keypoint& k)
{
	return k.p == this->p;
}


void c_keypoint::transform(const Eigen::Matrix4d& m) {
	p = m * p;
}
