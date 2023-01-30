#include "reprojection.h"
#include "core/threadCom.h"
#include "utils/coreutil.h"
#include "utils/imageutil.h"
#include "utils/visutil.h"
#include "VoxelGrid/kokkosFunctions.h"
#include "utils/kokkosUtil.h"

using namespace open3d;
using namespace std;

// KOKKOS_FUNCTION Kvec2f getCoordinate(Kvec3f p)
// {
// 	return Kvec2f(p.y * fy / p.z + ci, p.x * fx / p.z + cj);
// }
KOKKOS_FUNCTION bool withinLimits(const Kvec2f &c, const int &resi, const int &resj)
{
	return (c.x > 0 && c.x < resi - 1 && c.y > 0 && c.y < resj - 1);
}

//this is necessary since parallel_reduce will not work with multiple reducers on multiple host threads
struct summationstruct
{
	int count = 0;
	float err = 0;

	KOKKOS_DEFAULTED_FUNCTION
	summationstruct() = default;
	KOKKOS_DEFAULTED_FUNCTION
	summationstruct(const summationstruct &) = default;
	KOKKOS_FUNCTION
	summationstruct(const volatile summationstruct &source)
	{
		count = source.count;
		err = source.err;
	};
	KOKKOS_DEFAULTED_FUNCTION
	summationstruct &operator=(const summationstruct &) = default;

	KOKKOS_FUNCTION
	summationstruct operator+(const summationstruct &source) const
	{
		summationstruct tmp = *this;
		tmp.count += source.count;
		tmp.err += source.err;
		return tmp;
	}
	KOKKOS_FUNCTION
	summationstruct &operator+=(const summationstruct &source)
	{
		count += source.count;
		err += source.err;
		return *this;
	}
	KOKKOS_FUNCTION
	summationstruct operator+(const summationstruct &source) volatile const
	{
		summationstruct tmp = *this;
		tmp.count += source.count;
		tmp.err += source.err;
		return tmp;
	}
	KOKKOS_FUNCTION
	volatile summationstruct &operator+=(const volatile summationstruct &source) volatile
	{
		count += source.count;
		err += source.err;
		return *this;
	}
};

//The logic if the reprojection filter goes as follows:
//For each pair of projected points check if the depth, color and normal discrepancy is within limits (g_td, g_tc, g_tn)
//Only than we conclude that that a pair of pixel does not suffer from occulation and
//only than do we add the distance to the overall error
template <typename SpaceType>
void reprojectionfilter(SpaceType &space, pairTransform &trans)
{
	using namespace Kokkos;

	auto &matches = trans.filteredmatches;
	shared_ptr<Frame> f1;
	shared_ptr<Frame> f2;
	if (trans.k1->isChunk)
	{
		f1 = static_pointer_cast<Chunk>(trans.k1)->frames[0];
		f2 = static_pointer_cast<Chunk>(trans.k2)->frames[0];
	}
	else
	{
		f1 = static_pointer_cast<Frame>(trans.k1);
		f2 = static_pointer_cast<Frame>(trans.k2);
	}
	int resi = f1->lowpcd.extent(0);
	int resj = f1->lowpcd.extent(1);
	//look at coordiante system drawings for this to make sense.
	float ci = resi - g_lowIntr.intrinsic_matrix_(1, 2);
	float cj = g_lowIntr.intrinsic_matrix_(0, 2);
	float fx = g_lowIntr.intrinsic_matrix_(0, 0);
	float fy = g_lowIntr.intrinsic_matrix_(1, 1);

	//this is confirmed visualiy!
	// auto kabschtrans = eigentoView(trans.getTransformationFrom(trans.k1, trans.k2).cast<float>());
	auto kabschtrans = EigentoKmat4f(trans.getTransformationFrom(trans.k1, trans.k2).cast<float>());
	// local copies for lambda function. Dereferencing cpu pointers on gpu is not allowed!
	float td = g_td;
	float tc = g_tc;
	float tn = g_tn;
	auto local_lowpcdf1 = f1->lowpcd; //all shallow copies
	auto local_lowpcdf2 = f2->lowpcd;
	auto local_lowpcd_normalsf1 = f1->lowpcd_normals;
	auto local_lowpcd_normalsf2 = f2->lowpcd_normals;
	auto local_lowpcd_intensity1 = f1->lowpcd_intensity;
	auto local_lowpcd_intensity2 = f2->lowpcd_intensity;

	// Kokkos::View<float **, Kokkos::LayoutLeft, Kokkos::CudaSpace> test("test view", resi, resj);
	// cout << "allocated " << test.is_allocated() << endl;

	// float errsum = 0;
	// int countsum = 0;
	summationstruct sum;

	// Kokkos::fence();

	parallel_reduce(
		"Reprojection Filter Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {1, 1}, {resi - 1, resj - 1}), KOKKOS_LAMBDA(int i, int j, summationstruct &sum_) {
			if (local_lowpcdf1(i, j).z != 0)
			{
				// Kvec3f p1 = transformPoint(local_lowpcdf1(i, j), kabschtrans);
				Kvec3f p1 = kabschtrans() * local_lowpcdf1(i, j);
				Kvec2f coord = Kvec2f(p1.y * fy / p1.z + ci, p1.x * fx / p1.z + cj);
				if (withinLimits(coord, resi, resj))
				{
					int coordi = (int)coord.x;
					int coordj = (int)coord.y;
					Kvec3f p2 = local_lowpcdf2(coordi, coordj);
					if (p2.z != 0)
					{
						// Kvec3f n1 = transformNormal(local_lowpcd_normalsf1(i, j), kabschtrans);
						Kvec3f n1 = kabschtrans().rotate(local_lowpcd_normalsf1(i,j));
						float diff = (p2 - p1).length();
						float intensity = abs(local_lowpcd_intensity1(i, j) - local_lowpcd_intensity2(coordi, coordj));
						float angle = n1 * local_lowpcd_normalsf2(coordi, coordj);
						if (diff < td && intensity < tc && angle > tn)
						{
							//valid correspondence, not occulsion
							sum_.err += diff;
							sum_.count += 1;
						}
					}
				}
			}
		},
		sum);

	// if(f1->unique_id == 0 && f2->unique_id == 10){
	// 	//to visualize the pcds.
	// 	Kokkos::View<Kvec3f **> pcd1view("Pcd1 View", resi, resj);
	// 	parallel_for(
	// 		"Test Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
	// 			pcd1view(i, j) = transformPoint(local_lowpcdf1(i, j), kabschtrans);
	// 		});
	// 	auto pcd1 = ViewtoO3dPCd(pcd1view);
	// 	auto pcd2 = ViewtoO3dPCd(local_lowpcdf2);
	// 	pcd1->PaintUniformColor(Eigen::Vector3d(1.0,0.0,0.0));
	// 	pcd2->PaintUniformColor(Eigen::Vector3d(0.0,0.0,1.0));
	// 	visualization::DrawGeometries({pcd1, pcd2, getOrigin()});
	// }

	if (sum.count > 0)
		sum.err /= sum.count;
	// cout << "err_sum " << sum.err << " err count " << sum.count << endl;

	if (sum.err > g_reprojection_threshold || sum.count < g_lowj * g_lowi * 0.02)
	{
		matches = vector<match>();
		trans.set = false;
		utility::LogDebug("All matches removed by reprojection filter. Error was: {} Count was: {} \n", sum.err, sum.count);
		// cout << "All matches removed by reprojection filter. Error was: " << sum.err << " Count was: " << sum.count << endl;
		// cout << endl;
	}

#ifdef ENABLEASSERTIONS
	if (resi != g_lowi || resj != g_lowj)
	{
		cout << "Warning size in reprojection filter is wrong!\n";
	}
#endif
}
template void reprojectionfilter<Kokkos::Cuda>(Kokkos::Cuda &space, pairTransform &trans);
// template void reprojectionfilter<Kokkos::Cuda>(Kokkos::Cuda &space, shared_ptr<Frame> f1, shared_ptr<Frame> f2, pairTransform &trans);