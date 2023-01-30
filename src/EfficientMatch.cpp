#include "EfficientMatch.h"

std::ostream& operator<<(std::ostream& os, const rmatch& r)
{
	os << "frame index_1: " << r.fi1 << "; frame index_2: "<< r.fi2 << "; index_1: " << r.i1 << "; index_2: " << r.i2 << std::endl;;
	return os;
}