#include "match.h"
#include "core/Chunk.h"

using namespace std;
pairTransform &c_umap::operator()(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2)
{
	std::shared_ptr<KeypointUnit> tmp1; // = f1;
	std::shared_ptr<KeypointUnit> tmp2; //= f2;
	//unordered_map does not work with commutated pairs, even if the hash function we give is
	//commutative
	u_int64_t f1n = (u_int64_t)k1.get();
	u_int64_t f2n = (u_int64_t)k2.get();
	#ifdef ENABLEASSERTIONS
	if (f1n == f2n){
		cout << "Warning: You are trying to access the transformation between identical pointers!\n";
		cout << f1n << endl;
		cout << f2n << endl;
	}
	#endif
	if (f1n < f2n){
		tmp1 = k1;
		tmp2 = k2;
	}
	else {
		tmp1= k2;
		tmp2=k1;
	}
	//check if element already exists.
	if (x.find({tmp1, tmp2}) != x.end())
	{
#ifdef ENABLEASSERTIONS
		auto& pt = x[{tmp1, tmp2}];
		if (!(pt.k1.get() == tmp1.get() || pt.k1.get() == tmp2.get())){
			cout << "Error, saved pointer are not identical to access pointers 1\n";
		}
		if (!(pt.k2.get() == tmp1.get() || pt.k2.get() == tmp2.get()))
		{
			cout << "Error, saved pointer are not identical to access pointers 2\n";
		}
		return pt;
#else
		return x[{tmp1, tmp2}]; //return existing element
#endif
	} else{
		return emptyTransform; //element not existing, return emptyTransform
	}

}

// pairTransform& c_umap::operator()(std::shared_ptr<Chunk> c1, std::shared_ptr<Chunk> c2)
// {
// 	return (*this)(c1->frames.front(), c2->frames.front());
// }

pairTransform &c_umap::addElement(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2)
{

	std::shared_ptr<KeypointUnit> tmp1; // = f1;
	std::shared_ptr<KeypointUnit> tmp2; // = f2;
	u_int64_t f1n = (u_int64_t)k1.get();
	u_int64_t f2n = (u_int64_t)k2.get();
	if (f1n < f2n)
	{
		tmp1 = k1;
		tmp2 = k2;
	}
	else
	{
		tmp1 = k2;
		tmp2 = k1;
	}
	//pointer value is only used for acces, not for construction of pairTransform!
	auto pair = x.emplace(std::pair<std::shared_ptr<KeypointUnit>, std::shared_ptr<KeypointUnit>>(tmp1, tmp2), pairTransform(k1, k2));
	return pair.first->second; //first element of pair is iterator. Second of iterator pair is the value
}

void c_umap::removeElement(std::shared_ptr<KeypointUnit> k1, std::shared_ptr<KeypointUnit> k2)
{

	std::shared_ptr<KeypointUnit> tmp1; // = f1;
	std::shared_ptr<KeypointUnit> tmp2; // = f2;
	u_int64_t f1n = (u_int64_t)k1.get();
	u_int64_t f2n = (u_int64_t)k2.get();
	if (f1n < f2n)
	{
		tmp1 = k1;
		tmp2 = k2;
	}
	else
	{
		tmp1 = k2;
		tmp2 = k1;
	}
	//pointer value is only used for acces, not for construction of pairTransform!
	auto pair = x.erase(std::pair<std::shared_ptr<KeypointUnit>, std::shared_ptr<KeypointUnit>>(tmp1, tmp2));
}

// pairTransform& c_umap::addElement(std::shared_ptr<Chunk> c1, std::shared_ptr<Chunk> c2){
// 	return this->addElement(c1->frames.front(), c2->frames.front());
// }

Eigen::Matrix4d pairTransform::getTransformationFrom(std::shared_ptr<KeypointUnit> a, std::shared_ptr<KeypointUnit> b){

	if (a == k1 && b ==k2){
		return kabschtrans;
	} else if (a == k2 && b == k1){
		return invkabschtrans;
	} else{
		cout << "Warning: You are trying to access a transformation with wrong pointers!\n";
		return Eigen::Matrix4d::Identity();
	}
	#ifdef ENABLEASSERTIONS
		if (!set){
			std::cout << " Warning! Getting Transformation from unset pairTransform!!! (Once at programm start is okay)\n";
		}  
	#endif
}

std::ostream& operator<<(std::ostream& os, const match& r)
{
	os << "p1: " << r.p1 << "; p_2: "<< r.p2 << "; distance: " << r.d << "; indeces: " << r.indeces << std::endl;;
	return os;
}

