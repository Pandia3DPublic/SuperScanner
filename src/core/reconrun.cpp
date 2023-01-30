#include "reconrun.h"
#include "utils/matrixutil.h"
#include "postprocessing/postprocessing.h"
#include "cameras/CameraKinect.h"
#include "GlobalDefines.h"
#include "utils/visutil.h"
#include "utils/coreutil.h"
#include "filters/kabschfilter.h"
#include "filters/reprojection.h"
#include "filters/imufilter.h"
#include "core/Frame.h"
#include "core/Chunk.h"
#include "cmakedefines.h"
#include "genKps.h"
#include "genCors.h"
#include "integrate.h"
#include "cameras/CameraThreadandInit.h"
#include "cameras/DataCam.h"
#include "core/threadCom.h"
using namespace std;

//gets all chunks in the local group, e.g. frustums overlap and camera points in the same direction
//contains itself
vector<shared_ptr<Chunk>> getLocalGroup(Model &m, shared_ptr<Chunk> chunk)
{
	vector<shared_ptr<Chunk>> localGroup;
	localGroup.reserve(m.chunks.size());
	for (int i = 0; i < m.chunks.size(); i++)
	{
		m.chunks[i]->frustum->reposition(m.chunks[i]->chunktoworldtrans);
		if (chunk->frustum->inLocalGroup(*m.chunks[i]->frustum))
		{
			localGroup.push_back(m.chunks[i]);
		}
	}
	return localGroup;
}

vector<shared_ptr<Chunk>> getLimitedLocalGroup(const vector<shared_ptr<Chunk>> &localGroup, int ngroup, shared_ptr<Chunk> include)
{
	//limit to size of 30 random chunks
	int n = localGroup.size();
	if (n > ngroup)
	{
		vector<shared_ptr<Chunk>> tmpGroup;
		tmpGroup.reserve(ngroup);
		vector<bool> taken(n, 0);
		int ntaken = 0;
		while (ntaken < ngroup)
		{
			int r = rand() % n;
			// cout << r << endl;
			if (!taken[r])
			{
				taken[r] = true;
				tmpGroup.push_back(localGroup[r]);
				ntaken++;
			}
		}
		//mainly to include the current chunk
		if (include != nullptr)
		{
			//check if include already there
			bool found = false;
			for (int i = 0; i < tmpGroup.size(); i++)
			{
				if (tmpGroup[i] == include)
				{
					found = true;
				}
			}
			if (!found)
			{
				tmpGroup.pop_back();
				tmpGroup.push_back(include);
			}
		}
		return tmpGroup;
	}
	else
	{
		return localGroup;
	}
}

//resets all global thread vars
void resetThreadVars()
{
	pandia_integration::stopintegrating = false;
}

int reconrun(Model &m, bool integrate)
{

	Kokkos::Cuda reconSpace = SpaceInstance<Kokkos::Cuda>::create();

	using namespace open3d;
	srand(1); //seed for reasonable testing (local groups)

	//############################## variable declarations ##############################
	//solverWrapper solver(&m);
	//thread vars, note that some are in threadcom just for ui debug
	threadCom::g_reconThreadFinished = false;
	threadCom::g_current_slam_finished = false;
	thread integrationThread;
	std::atomic<bool> stopRecording(false);
	shared_ptr<std::thread> cameraThread;

	//debug variables
	int npass = 0; //number of cors passed through kabsch (chunk)
	int npassm = 0;
	//utitlity variables
	std::shared_ptr<Frame> f;
	vector<int> nvalidFrameIndices;
	int nnv = 0;				//number of invalid frames in consequtive order
	int cfi = 0;				//current frame index
	int cci = 0;				//current chunk index
	bool completeChunk = false; //  indicates that the last chunk is full and valid
	bool firstframe = true;
	bool firstchunk = true;
	bool lastChunkValid = true;
	shared_ptr<Frame> lastframe;

	shared_ptr<Chunk> currentChunk;
	auto timeofLastGlobalOpt = std::chrono::high_resolution_clock::now();

	//############################## camera or connection thread start ##############################
	if (!threadCom::g_cameraParameterSet)
	{
		utility::LogError("Cam paras must be set before starting camera thread in reconrun. \n");
	}

	cout << "before data thread start \n";
	// Camera Kinect
	if (g_camType == camtyp::typ_kinect)
	{
		cameraThread = make_shared<std::thread>(KinectThread, std::ref(stopRecording), std::ref(threadCom::g_cameraParameterSet)); //startet neuen thread und liest bidler ein. Filtert bilder und schreibt diese in picturebuffer
	}

	if (g_camType == camtyp::typ_data)
	{
		cout << "starting data cam threads \n";
		cameraThread = make_shared<std::thread>(DataCamThreadFunction, std::ref(stopRecording)); //read images from hardrive
	}

	//#################################### start various threads
	//start threads
	if (integrate)
		integrationThread = thread(pandia_integration::integrateThreadFunction, &m);

	int readUntil = g_nread;
	Eigen::Matrix4d Tc = Eigen::Matrix4d::Zero(); //chunk matrix for early integrate

	//######################################################### main loop ##################################################
	int i = 0;
	Kokkos::Timer reconTimer;
	double lasttime = reconTimer.seconds();
#ifdef WITHOUT_GUI
	std::cout << "WITHOUT_GUI is defined!\n";
	while (i < readUntil)
	{
#else
	while (!threadCom::g_clear_button) // && !threadCom::g_postProcessing)
	{
#endif
		g_reconIterationTimer = reconTimer.seconds() - lasttime; //get time of last iteration
		lasttime = reconTimer.seconds();
		Kokkos::Timer chunkloopTimer;
		// cout << g_reconIterationTimer << endl;
		//Timer t("Time in single iteration",TimeUnit::millisecond);
		//if number of read frames exceeds nread put in pause state
		if (i >= readUntil)
		{
			threadCom::g_pause = true;
			threadCom::g_programState = threadCom::gui_PAUSE;
			readUntil = readUntil + g_nread;
		}
		#ifdef USETIMER
		PandiaTimer t("generate new chunk");
		#endif
		//#######################################generate new chunk if necessary ##############################################
		//generate new chunk if necessary
		if (firstframe)
		{
			currentChunk = make_shared<Chunk>();
			firstframe = false;
		}
		if (completeChunk)
		{
			currentChunk = make_shared<Chunk>();
			if (lastChunkValid)
			{
				currentChunk->frames.push_back(lastframe); // put a copy of the last frame into the new chunk.
				lastframe->structureIndex=0;
				threadCom::g_trackingLost = false;
			}
			else
			{
				threadCom::g_trackingLost = true;
			}
			nvalidFrameIndices.clear(); //this is new
			nnv = 0;
			completeChunk = false;
		}
		if (nnv > 5)
		{
			// utility::LogWarning("Restarted Chunk since tracking was lost for 6 frames within the chunk\n");
			cout << "Restarted Chunk since tracking was lost for 6 frames within the chunk\n";
			threadCom::g_trackingLost = true;
			for (auto &f : currentChunk->frames)
			{
				if (f->pushedIntoIntegrationBuffer)
				{
					pandia_integration::removeFrameFromIntegration(f);
				}
			}
			currentChunk = make_shared<Chunk>();
			nvalidFrameIndices.clear();
			nnv = 0;
			completeChunk = false;
			lastChunkValid = false;
		}
		#ifdef USETIMER
		t.PrintandReset("aquire new data");
		#endif
		//########################################### acquire new chunk data #########################################
		//get image data
		f = getSingleFrame(m.recordbuffer); //this methods locks if no frame is available
		if (f == nullptr)
			goto pauseStateLabel; //happens if pause gets us out of getSingleFrame
		//check if image data gives enough orb keypoints
		if (f->orbKeypoints.size() < 50)
		{
			threadCom::g_trackingLost = true;
			utility::LogDebug("warning: low keypoint size : {} \n", f->orbKeypoints.size());
			while (f->orbKeypoints.size() < 10 && i < g_nread - 1 && !threadCom::g_clear_button)
			{
				f = getSingleFrame(m.recordbuffer); //this methods locks if no frame is available
				if (f == nullptr)
					goto pauseStateLabel; //happens if pause gets us out of getSingleFrame
				i++;
				// utility::LogWarning("Discarded Frame since it has less than 10 keypoints.\n");
				cout << "Discarded Frame since it has less than 10 keypoints.\n";
				cout << "Discarded Frame id " << f->unique_id << endl;
				while (threadCom::g_pause)
				{
					std::this_thread::sleep_for(20ms);
				};
			}
			threadCom::g_trackingLost = false;
		}
		#ifdef USETIMER
		t.PrintandReset("put frame in place");
		#endif
		//############################################### put frame in place #################################
		//put the new frame in the correct place
		if (nvalidFrameIndices.size() != 0)
		{ //insert new frame if invalid frame was detected. Note temporal realtions are not preserved this way
			// utility::LogWarning("Refilling invalid frame in chunk at position number {}. \n", nvalidFrameIndices.front());
			cout << "Refilling invalid frame in chunk at position number " << nvalidFrameIndices.front() << "\n";
			utility::LogDebug("nnv: {} \n", nnv);
			//make sure the frame is deintegrated or not integrated to start with
			if (currentChunk->frames[nvalidFrameIndices.front()]->pushedIntoIntegrationBuffer)
				pandia_integration::removeFrameFromIntegration(currentChunk->frames[nvalidFrameIndices.front()]);
			currentChunk->frames[nvalidFrameIndices.front()] = f;
			cfi = nvalidFrameIndices.front();
			nvalidFrameIndices.erase(nvalidFrameIndices.begin()); //pop_front does not exist for std::vector
		}
		else
		{
			currentChunk->frames.push_back(f); //frame is inserted but not yet valid. usefull for high res filter
			cfi = currentChunk->frames.size() - 1;
		}
		f->structureIndex = cfi;

		#ifdef USETIMER
		t.PrintandReset("core filter operations");
		#endif

		//################################ core filter operations #####################################
		//int npass = 0;
		//constant effort, scales well
		for (int j = 0; j < currentChunk->frames.size(); j++)
		{
			if (j != cfi)
			{
				auto &other = currentChunk->frames[j];
				auto &ptransform = currentChunk->pairTransforms.addElement(other, f);
				getCors(ptransform);
				kabschfilter(ptransform);
				if (ptransform.set)
				{
					reprojectionfilter(reconSpace, ptransform);
					// reprojectionfilter(reconSpace, other, f, ptransform);
				}
			}
		}
		#ifdef USETIMER
		t.PrintandReset("integrate frame");
		#endif
		//cout << "npass " << npass << endl;
		//######################## integrate frame #######################################
		if (firstchunk)
		{
			Tc = Eigen::Matrix4d::Identity();
		}
		else
		{
			if (lastChunkValid)
				Tc = m.chunks[m.chunks.size() - 1]->frames.back()->getFrametoWorldTrans(); //-1 here since the new chunk isnt added yet
		}
		if (lastChunkValid && currentChunk->doFrametoModelforNewestFrame())
		{
			Eigen::Matrix4d T = Tc * f->chunktransform;
			f->setFrametoWorldTrans(T);
			pandia_integration::integrationlock.lock();
			pandia_integration::integrationBuffer.push_back(f);
			pandia_integration::integrationlock.unlock();
			f->pushedIntoIntegrationBuffer = true;
			//if (m.chunks.size() == 11) {
			//cout << f->unique_id << "getting integrated before opt \n";
			//}
			//	cout << "unique id " << f->unique_id << endl;
			//visualization::DrawGeometries({geometry::PointCloud::CreateFromRGBDImage(*f->rgbd,g_intrinsic)});
			//	}
			threadCom::g_currentposlock.lock();
			m.currentPos = T;
			threadCom::g_currentposlock.unlock();
		}
		#ifdef USETIMER
		t.PrintandReset("chunk is full check if frames have cors");
		#endif

		//#####################if chunk is full check if all frames have cors, otherwise remove frame ############################
		if (currentChunk->frames.size() == g_nchunk && nvalidFrameIndices.size() == 0)
		{
			completeChunk = checkValid(currentChunk, nvalidFrameIndices);
			if (completeChunk)
			{ //else jumps very far
				utility::LogDebug("Chunk number {} is full \n", m.chunks.size() + 1);
				if (currentChunk->OptimizeAlignment(getDoffromKabschChunk(currentChunk)))
				{ //high res was active, so check again for valid chunk
					completeChunk = checkValid(currentChunk, nvalidFrameIndices);
				}
				if (!completeChunk)
				{
					utility::LogError("Frame is invalid after high res filter \n");
					nnv += nvalidFrameIndices.size();
				}
				#ifdef USETIMER
				t.PrintandReset("Model lastframe and new chunk keypoints");
				#endif

				//######################################## Model  ################################################################################
				if (completeChunk)
				{
					nnv = 0; //reset bad frame counter
					lastframe = make_shared<Frame>();
					Frame::nGPUFrames--;
					lastframe->duplicate = true;
					lastframe->orbKeypoints = currentChunk->frames.back()->orbKeypoints;
					// lastframe->orbDescriptors = currentChunk->frames.back()->orbDescriptors.clone();
					lastframe->orbDescriptors = currentChunk->frames.back()->orbDescriptors; //shallow copy
					lastframe->unique_id = currentChunk->frames.back()->unique_id; //same id, not sure if good
					frame_id_counter.id--;
					lastframe->imuVector = currentChunk->frames.back()->imuVector;
					lastframe->lowpcd = currentChunk->frames.back()->lowpcd; //shallow copy
					lastframe->lowpcd_h = currentChunk->frames.back()->lowpcd_h; //shallow copy
					lastframe->lowpcd_normals = currentChunk->frames.back()->lowpcd_normals; //shallow copy
					lastframe->lowpcd_intensity = currentChunk->frames.back()->lowpcd_intensity; //shallow copy
					lastframe->rgb = currentChunk->frames.back()->rgb;
					lastframe->depth = currentChunk->frames.back()->depth;
					// lastframe->timestamp = currentChunk->frames.back()->timestamp;
					// lastframe->lowpcd = make_shared<open3d::geometry::PointCloud>(*(currentChunk->frames.back()->lowpcd));
					// do not shallow copy views, so that an error gets thrown if they are used.
					currentChunk->generateEfficientKeypoints(); //for keypoint merging
					currentChunk->generateChunkKeypoints(); // must happen after lastframe was build
					currentChunk->generateFrustum();

					currentChunk->deleteStuff();	  //todo implement
					m.chunks.push_back(currentChunk); //add chunk to model
					cci = m.chunks.size() - 1;
					currentChunk->structureIndex = cci;
					utility::LogDebug("model stuff with chunk number {} \n", cci + 1);
					#ifdef USETIMER
					t.PrintandReset("local group");
					#endif

					//################################# build local group ###################################
					//note: new chunk is already in model
					vector<shared_ptr<Chunk>> localGroup;
					if (lastChunkValid && m.chunks.size() > 5)
					{ //last chunk valid
						currentChunk->frustum->reposition(m.chunks[m.chunks.size() - 2]->frames.back()->getFrametoWorldTrans());
						localGroup = getLocalGroup(m, currentChunk); //new chunk might have identity trans here
						if (Frame::nGPUFrames > g_maxGPUFrames)
						{
							auto localMemoryGroup = getLimitedLocalGroup(localGroup, g_maxGPUFrames / (g_nchunk - 1), currentChunk);
							srand(m.chunks.size()); //reset random generator so results are independent of streaming rate
							m.StreamNonLGtoCPU(localMemoryGroup);
						}
						localGroup = getLimitedLocalGroup(localGroup, g_nLocalGroup);
						//if (m.chunks.size() == 60)
						// visualization::DrawGeometries({getOrigin(), getvisFrusti(m,*currentChunk->frustum)});
					}
					else
					{
						localGroup = m.chunks; //todo dont copy here
					}
					#ifdef USETIMER
					t.PrintandReset("rest");
					#endif
					//############################################# do all the filters for chunks ####################################
					utility::LogDebug("Before Chunk filters \n");
					// cout << "localgroup size: " << localGroup.size() << endl;
					npassm = 0;
					int Mind;
					for (int j = 0; j < localGroup.size(); j++)
					{
						if (localGroup[j] != currentChunk)
						{

							Mind = localGroup[j]->structureIndex;
							auto &other = localGroup[j];
							auto &ptransform = m.pairTransforms.addElement(other, currentChunk);
							getCors(ptransform);
							kabschfilter(ptransform);
							if (ptransform.set)
							{
								reprojectionfilter(reconSpace, ptransform); //this can cause false negatives. todo
							}
							if (ptransform.set && lastChunkValid)
							{
								jumpingFilter(ptransform, m.chunks[m.chunks.size() - 2]->frames.back());
								// npassm += 1;
							}
						}
					}
					utility::LogDebug("After Chunk filters \n");

					bool ChunkhasCors = checkValid(m); //note structure is such that this can change in high res and imu filters.
					utility::LogDebug("local group size: {} \n", localGroup.size());
					//############################################# do optimization ################################################
					if (!firstchunk)
					{
						if (ChunkhasCors)
						{ //note this extra check if just to keep the structure in case we need to high res filter here.
							cout << "Chunk number in opt " << m.chunks.size() << "\n";
							m.doChunktoModelforNewestChunk();
							cout << "################################ \n";
							// cout << currentChunk->chunktoworldtrans.block<3,1>(0,3) << endl << endl;
							// m.performSparseOptimization(getInitialDofs(m));
							// if(!checkValid(m)){ //todo only check if high res was used
							// 	utility::LogError("Chunk number {} is invalid after high res filter \n", cci + 1);
							// 	ChunkhasCors = false;
							// }
							pandia_integration::integrationlock.lock(); //for thread safety. Changing position during integration can result in invalid matrices (has never actually happended)
							currentChunk->setWorldTransforms();
							pandia_integration::integrationlock.unlock();
						}

						//############################################# Check with IMU Filter ################################################

						//####################################### chunk is accepted ######## set world trans and mark frames for integration and update reintegration buffer
						if (ChunkhasCors)
						{
							utility::LogDebug("Chunk added to model \n");
							lastChunkValid = true;

							//integration ##################
							pandia_integration::integrationlock.lock(); //for thread safety. Changing position during integration can result in invalid matrices (has never actually happended)
							m.setWorldTransforms();						//todo only necessary if non linear opt was performed
							for (int j = 0; j < g_nchunk; j++)
							{
								auto &f = m.chunks.back()->frames[j];
								if (!f->duplicate && !f->pushedIntoIntegrationBuffer)
								{
									pandia_integration::integrationBuffer.push_back(f);
								}
							}
							pandia_integration::updateReintegrationBuffer();
							pandia_integration::integrationlock.unlock();
						}
						else
						{ //no valid new chunk
							lastChunkValid = false;
							if (m.chunks.size() > 1)
							{ //mature model
								// m.invalidChunks.push_back(m.chunks.back());
								//erase chunk from model
								for (int j = 0; j < m.chunks.back()->frames.size(); j++)
								{
									if (m.chunks.back()->frames[j]->pushedIntoIntegrationBuffer)
										pandia_integration::removeFrameFromIntegration(m.chunks.back()->frames[j]);
								}
								m.chunks.erase(m.chunks.end() - 1);
								utility::LogDebug("Put chunk on invalid list \n");
							}
							else
							{	//young model kill everything todo make this stable for live integration
								//m = Model();
								//m.invalidChunks = vector<shared_ptr<Chunk>>();
								//firstframe = true;
								//firstchunk = true;
								//nnv = 0;
								//nvalidFrameIndices.clear();
								//cci = 0;
								//cfi = 0;
								//completeChunk = false;
								//frame_id_counter.id = 0;
								//utility::LogWarning("First Chunks not all valid. Restarted Model! \n");
							}
						}
					}
					else
					{ //its the first chunk
						firstchunk = false;
						lastChunkValid = true;
						utility::LogDebug("First Chunk added to model \n");
						m.chunks[0]->chunktoworldtrans = getIdentity();
						pandia_integration::integrationlock.lock();
						for (int k = 0; k < m.chunks[0]->frames.size(); k++)
						{
							auto &f = m.chunks[0]->frames[k];
							f->setFrametoWorldTrans(f->chunktransform);
							f->worldtransset = true;
						}
						for (int k = 0; k < m.chunks[0]->frames.size(); k++)
						{
							if (!m.chunks[0]->frames[k]->pushedIntoIntegrationBuffer)
							{
								pandia_integration::integrationBuffer.push_back(m.chunks[0]->frames[k]);
							}
						}
						pandia_integration::updateReintegrationBuffer();
						pandia_integration::integrationlock.unlock();
					}

					utility::LogDebug("End Model stuff \n");
				}
			}
			else
			{
				nnv += nvalidFrameIndices.size();
				for (int j = 0; j < nvalidFrameIndices.size(); j++)
				{
					// utility::LogWarning("Frame number {} is invalid\n", nvalidFrameIndices[j]);
					cout << "Frame number " << nvalidFrameIndices[j] << " is invalid\n";
				}
			}
			g_reconChunkItTimer = chunkloopTimer.seconds();
		}
		i++;

	pauseStateLabel:
		bool frames_removed_inPause = false;

		while (threadCom::g_pause)
		{

			//remove kabsch integrated frames
			if (!frames_removed_inPause)
			{
				if (currentChunk->frames.size() != g_nchunk || nvalidFrameIndices.size() != 0)
				{
					for (auto f : currentChunk->frames)
					{
						if (f->pushedIntoIntegrationBuffer)
						{
							pandia_integration::removeFrameFromIntegration(f);
						}
					}
				}
				frames_removed_inPause = true;

				// cout << "hi \n";
				// m.performSparseOptimization(getInitialDofs(m));
				// //reintegrate everything that has changed. Integration thread has stopped here
				// pandia_integration::integrationlock.lock(); //still use locks just in case
				// m.setWorldTransforms();
				// pandia_integration::updateReintegrationBuffer();
				// pandia_integration::integrationlock.unlock(); //still use locks just in case
				// cout << "size of reint buffer after opt " << pandia_integration::reintegrationBuffer.size() << endl;
			}

			std::this_thread::sleep_for(20ms);
		}
	}

	pandia_integration::integrationlock.lock();
	m.setWorldTransforms(); //superflous if no new global solve after slam finish
	pandia_integration::updateReintegrationBuffer();
	pandia_integration::integrationlock.unlock();

	threadCom::g_current_slam_finished = true;

	//remove the frames which have been integrated last and don't fill a chunk
	if (currentChunk->frames.size() != g_nchunk || nvalidFrameIndices.size() != 0)
	{
		for (auto f : currentChunk->frames)
		{
			if (f->pushedIntoIntegrationBuffer)
			{
				pandia_integration::removeFrameFromIntegration(f);
			}
		}
	}

	//################## ending threads ######################
	stopRecording = true;
	cameraThread->join();

	pandia_integration::stopintegrating = true;
	if (integrate)
		integrationThread.join();

	//reset

	m.recordbuffer.clear(); //only ever used in pause mode, so clearing is allowed
	threadCom::g_framebuffer.clear();
	//reset

	std::cout << "All recon threads joined \n";

	resetThreadVars(); //for possible next launch


	SpaceInstance<Kokkos::Cuda>::destroy(reconSpace);
	threadCom::g_reconThreadFinished = true;

	return 0;
}