#include "integrate.h"
#include "utils/matrixutil.h"
#include "threadCom.h"

using namespace open3d;
using namespace std;

//variable declaration for static variables
std::atomic<bool> pandia_integration::stopintegrating(false); //signaling bool to stop the integration thread
list <shared_ptr<Frame>> pandia_integration::integrationBuffer; //buffer that contains new frames that must be integrate 
list <shared_ptr<Frame>> pandia_integration::reintegrationBuffer; //buffer that contains new frames that must be integrate 
list <shared_ptr<Frame>> pandia_integration::deintegrationBuffer; //buffer that contains frames that are not valid and must be deintegrated
list <shared_ptr<Frame>> pandia_integration::integratedframes; //a vector that gets periodically sorted which contains the frames in order corresponding to the dif in integrateddofs and worlddofs
std::mutex pandia_integration::integrationlock; //integrationlock for all integration buffers
std::mutex pandia_integration::tsdfLock; //lock for opengl data transfer
pandia_integration::comps pandia_integration::comparator;


//this is called in a seperate thread
//Note: Thread safety is super important for all position variables!


void pandia_integration::removeFrameFromIntegration(shared_ptr<Frame> f) {
	bool found = false;
	pandia_integration::integrationlock.lock();
	for (auto it = pandia_integration::integrationBuffer.begin(); it!= pandia_integration::integrationBuffer.end(); it++) {
		if (f == (*it)) {
			pandia_integration::integrationBuffer.erase(it);
			found = true;
			break;
		}
	}
	//frame was already integrated, need to deintegrate
	if (!found) {
		for (auto it = pandia_integration::integratedframes.begin(); it!= pandia_integration::integratedframes.end(); it++) {
			if (f == (*it)) {
				pandia_integration::deintegrationBuffer.push_back(f);
				break;
			}
		}
	}
	pandia_integration::integrationlock.unlock();
	f->pushedIntoIntegrationBuffer = false;
}


void pandia_integration::integrateThreadFunction(Model* m) {
	// Kokkos::initialize();
	Kokkos::Cuda integrateSpace = SpaceInstance<Kokkos::Cuda>::create();

	std::cout << "Start integration Thread \n";
	shared_ptr<Frame> fint; //integration frame
	shared_ptr<Frame> fre; //reintegration frame
	shared_ptr<Frame> fde; //deintegration frame
	Eigen::Matrix4d tint = Eigen::Matrix4d::Identity(); //integration transformation
	Eigen::Matrix4d tre= Eigen::Matrix4d::Identity(); //reintegration transformation
	Eigen::Matrix4d tde= Eigen::Matrix4d::Identity(); //deintegration transformation
	Eigen::Vector6d worlddofsint = Eigen::Vector6d::Zero(); //worlddofs for thread safety
	Eigen::Vector6d worlddofsre= Eigen::Vector6d::Zero(); //worlddofs for thread safety
	while (!stopintegrating) { //while not stopped do integration and reintegration
		// if nothing happens this thread goes to sleep for 20 ms
		bool nonew = true;
		bool noreint = true;
		bool nodeint = true;
		//################ thread sensity preperation ######################
		integrationlock.lock();
		bool emptyint = integrationBuffer.empty();
		bool emptyre = reintegrationBuffer.empty();
		bool emptyde = deintegrationBuffer.empty();
		if (!emptyint) {
			fint = integrationBuffer.front();
			integrationBuffer.pop_front();
			tint = fint->getFrametoWorldTrans();
			worlddofsint = fint->getWorlddofs();
			integratedframes.push_back(fint);//do push_back here to avoid race condition with frame removal
		}
		if (!emptyre) {
			fre = reintegrationBuffer.front();
			reintegrationBuffer.pop_front();
			tre = fre->getFrametoWorldTrans();
			worlddofsre = fre->getWorlddofs();

		}
		if (!emptyde) {
			fde = deintegrationBuffer.front();
			deintegrationBuffer.pop_front();
			tde = fde->getFrametoWorldTrans();
			integratedframes.remove(fde);
			
		}
		integrationlock.unlock();
		//################################## end thread sensity preperation #################### 
		//################################## main loop part #################### 
		//################################## integration #################### 
		if (!emptyint) {
			//shold not need lock since this data is never changed
			tsdfLock.lock();
			m->globalGrid->integrate(integrateSpace, fint, tint.cast<float>());
			tsdfLock.unlock();

			integrationlock.lock();
			fint->integrateddofs = worlddofsint; //lock should be unnecessary
			fint->integrated =true;
			integrationlock.unlock();
			nonew = false;
		}
//######################################## reintegration #####################################
		if (!emptyre) {
			//reintegrate here
			tsdfLock.lock();
			m->globalGrid->deIntegrate(integrateSpace, fre, getT(fre->integrateddofs.data()).cast<float>());
			m->globalGrid->integrate(integrateSpace, fre, tre.cast<float>());

			tsdfLock.unlock();

			integrationlock.lock();
			fre->integrateddofs = worlddofsre;
			integrationlock.unlock();
			noreint = false;
			//stream out of gpu memory since it's unlikely to need reintegration soon
			//start streaming out here before local group hard cap
			if(Frame::nGPUFrames > 0.8*g_maxGPUFrames){ 
				fre->MovetoCPU();
			}
		}
//######################################## deintegration #####################################

		if (!emptyde) {
			//deintegrate here
			tsdfLock.lock();
			m->globalGrid->deIntegrate(integrateSpace, fde, getT(fde->integrateddofs.data()).cast<float>());
			tsdfLock.unlock();
			integrationlock.lock();
			fde->integrateddofs = Eigen::Vector6d::Zero();
			integratedframes.remove(fde);
			integrationlock.unlock();

			nodeint = false;
			//stream out of gpu memory, since its not needed there
			fde->MovetoCPU();
		}
		//Frames which are deintegrated are deleted, so no streaming to cpu is necessary
		if (nonew && noreint && nodeint) {
			std::this_thread::sleep_for(20ms); //sleep since no task is necessary
		}

#ifdef ENABLEASSERTIONS
		if (tint(3, 0) != 0 || tint(3, 1) != 0 || tint(3, 2) != 0 || tint(3, 3) != 1){
			cout << "Warning!!! integration matrix tint is invalid! \n";
		}
		if (tre(3, 0) != 0 || tre(3, 1) != 0 || tre(3, 2) != 0 || tre(3, 3) != 1)
		{
			cout << "Warning!!! integration matrix tre is invalid! \n";
		}
		if (tde(3, 0) != 0 || tde(3, 1) != 0 || tde(3, 2) != 0 || tde(3, 3) != 1)
		{
			cout << "Warning!!! integration matrix tde is invalid! \n";
		}
#endif
	}
	//####################################################### stop called ##########################################
	//do rest after stop signal 
	cout << "stopped intergration main loop \n";
	integrationlock.lock();
	while (!integrationBuffer.empty()) {
		auto f = integrationBuffer.front();
		integrationBuffer.pop_front();
		cout << "integration after stop called \n";
		tsdfLock.lock();
		m->globalGrid->integrate(integrateSpace, f, f->getFrametoWorldTrans().cast<float>());
		tsdfLock.unlock();
		f->integrateddofs = f->getWorlddofs();
		f->integrated =true;
		integratedframes.push_back(f);
	}

	while (!reintegrationBuffer.empty()) {
		//reintegrate here
		cout << "reintegrate final " << reintegrationBuffer.size() << "\n";
		shared_ptr<Frame> f = reintegrationBuffer.front();
		reintegrationBuffer.pop_front();

		tsdfLock.lock();
		m->globalGrid->deIntegrate(integrateSpace, f, getT(f->integrateddofs.data()).cast<float>());
		m->globalGrid->integrate(integrateSpace, f, f->getFrametoWorldTrans().cast<float>());
		tsdfLock.unlock();

		f->integrateddofs = f->getWorlddofs();

	}

	while (!deintegrationBuffer.empty()) {
		//reintegrate here
		utility::LogInfo("deintegrate final {} \n", deintegrationBuffer.size());
		shared_ptr<Frame> f = deintegrationBuffer.front();
		deintegrationBuffer.pop_front();
		tsdfLock.lock();
		m->globalGrid->integrate(integrateSpace, f, getT(f->integrateddofs.data()).cast<float>());
		tsdfLock.unlock();
		f->integrateddofs = f->getWorlddofs();
		integratedframes.remove(f);

	}

	integrationlock.unlock();

	std::cout << "Finished integrating \n";

}

double pandia_integration::diff(shared_ptr<Frame> a) {
	Eigen::Vector6d diff;
	diff.block<3, 1>(0, 0) = a->getWorlddofs().block<3, 1>(0, 0) - a->integrateddofs.block<3, 1>(0, 0);
	diff.block<3, 1>(3, 0) = 2 * (a->getWorlddofs().block<3, 1>(3, 0) - a->integrateddofs.block<3, 1>(3, 0));
	return diff.norm();
}

bool pandia_integration::compFrames(shared_ptr<Frame> a, shared_ptr<Frame> b){
	return diff(a) > diff(b);
}

// must always be called in mutex with integrationlock
void pandia_integration::updateReintegrationBuffer() {
	if (!integratedframes.empty()) {
		integratedframes.sort(compFrames);
		reintegrationBuffer.clear();
		auto it = integratedframes.begin();
		for (int i = 0; i < integratedframes.size(); i++) {
			if (diff(*it ) < g_treint) break; //must check after range check in loop head
			reintegrationBuffer.push_back(*it);
			it++;
		}
	}
}



