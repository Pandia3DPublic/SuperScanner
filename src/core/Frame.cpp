#include "Frame.h"
#include "utils/matrixutil.h"
#include "utils/coreutil.h"
#include "core/threadCom.h"
#include "utils/imageutil.h"


using namespace open3d;

std::atomic<int> Frame::nGPUFrames{0};
Kokkos::View<unsigned char **[3]> Frame::rgbDeallocHelper;
Kokkos::View<unsigned short **> Frame::depthDeallocHelper;

Frame::Frame() {
	//utility::SetVerbosityLevel(utility::VerbosityLevel::VerboseAlways);
	 //utility::LogDebug("blubberdibu");
	unique_id = frame_id_counter;
	nGPUFrames ++; 
	isChunk = 0;
}

Frame::~Frame() {
	if(dataOnGPU && !duplicate){
		nGPUFrames--;
	}

}

void Frame::MovetoCPU(){
	viewLock.lock();
	if (dataOnGPU) {
		// cout << unique_id << endl;
		Kokkos::deep_copy(rgb_h,rgb);
		rgb = Frame::rgbDeallocHelper; //assign to other view so that deallocation happens

		Kokkos::deep_copy(depth_h, depth);
		depth = Frame::depthDeallocHelper;

		nGPUFrames--;
		dataOnGPU = false;
	}
	viewLock.unlock();
}
void Frame::MovetoGPU(){
	viewLock.lock();
	if (!dataOnGPU)
	{
		rgb = Kokkos::View<unsigned char[G_RESI][G_RESJ][3]>("RgbFrame_View");
		Kokkos::deep_copy(rgb, rgb_h);

		depth = Kokkos::View<unsigned short[G_RESI][G_RESJ]>("DepthFrame_View");
		Kokkos::deep_copy(depth, depth_h);

		nGPUFrames++;
		dataOnGPU = true;
		// cout << "moved data to gpu! "  << unique_id << " \n";
		// visualization::DrawGeometries({ViewtoImage(rgb)});
	}
	viewLock.unlock();
}

Eigen::Vector6d Frame::getWorlddofs() {
	return worlddofs;
}

//only ever call this in the reconthread wiht lock
Eigen::Matrix4d Frame::getFrametoWorldTrans() {
	return frametoworldtrans;
}
//must be called in integrationlock (todo)
void Frame::setFrametoWorldTrans(const Eigen::Matrix4d& a) {
	frametoworldtrans = a;
	worlddofs = MattoDof(a);
}

void Frame::setworlddofs(const Eigen::Vector6d& a) {
	worlddofs = a;
	frametoworldtrans = getT(a.data());
}





