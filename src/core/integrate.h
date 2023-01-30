#pragma once
#include "core/Frame.h"
#include "core/Model.h"

//basically a singleton class
class pandia_integration {
public:

	static void updateReintegrationBuffer();
	//note: no inverse needed for the cuda variant of integrated. happens internally
	static void integrateThreadFunction(Model* m);
	static double diff(std::shared_ptr<Frame> a);
	static bool compFrames(std::shared_ptr<Frame> a, std::shared_ptr<Frame> b);
	static void removeFrameFromIntegration(std::shared_ptr<Frame> f);

	static std::atomic<bool> stopintegrating; //signaling bool to stop the integration thread
	static std::list<std::shared_ptr<Frame>> integrationBuffer; //buffer that contains new frames that must be integrate
	static std::list <std::shared_ptr<Frame>> reintegrationBuffer; //buffer that contains new frames that must be integrate
	static std::list<std::shared_ptr<Frame>> deintegrationBuffer;  //buffer that contains frames that are not valid and must be deintegrated
	static std::list<std::shared_ptr<Frame>> integratedframes;	   //a vector that gets periodically sorted which contains the frames in order corresponding to the dif in integrateddofs and worlddofs

	static std::mutex integrationlock; //integrationlock for position data
	static std::mutex tsdfLock; //lock for real time meshing of tsdf

	struct comps {
		bool operator()(std::shared_ptr<Frame> a, std::shared_ptr<Frame> b)
		{
			return diff(a) > diff(b);
		}
	};

	static comps comparator;

};

