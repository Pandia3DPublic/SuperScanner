//this files contains atomic varialbes, mutex locks and enums that are used for thread communication
//all varialbes are sorted by topic (integration, general, camera, etc)
//thread relevant gui variables should be placed here
//this file should be included in case communication variables are needed.
//additionally config variables are stored here
//additionally other global varialbes are stored here that are not thread critical.
#pragma once
#include "match.h"
#include "core/Model.h"

struct RawFrameDataPointers
{
	std::shared_ptr<k4a::capture> capture;
	std::shared_ptr<std::vector<k4a_imu_sample_t>> imuVector;
};

namespace threadCom{

//camera variables  ------------------------------------------
extern std::atomic<bool> g_cameraConnected; //newly moved
extern std::atomic<bool> g_take_dataCam; //take hardrive data
extern k4a::device g_deviceKinect;
extern k4a::calibration g_calibrationKinect;
extern std::atomic<bool> g_cameraParameterSet;
extern std::atomic<bool> g_warmupCamera;
//only use this in g_bufferlock
extern std::list<std::shared_ptr<Frame>> g_framebuffer; //the camera thread adds frames here for reconrun
extern std::list<RawFrameDataPointers> g_rawbuffer;

//config variables  ------------------------------------------

//meshing variables  ------------------------------------------
extern std::atomic<bool> unmeshed_data; //indicates wheter marching cubes has run after integrate or deintegrate.

//tracker variables  ------------------------------------------
extern std::mutex g_currentposlock; //just for the current position
extern std::atomic<bool> g_current_slam_finished;
extern std::atomic<bool> g_trackingLost;
extern std::atomic<bool> g_reconThreadFinished;
//postprocessing variables  ------------------------------------------
extern std::atomic<bool> g_postProcessing; //sets gui block and pp indicator in gui

//integration variables  ------------------------------------------

//non-linear solver variables  ------------------------------------------
extern std::mutex g_solverlock; //gpu solver lock
extern std::atomic<bool> g_solverRunning;//indicates that the global solver is running. is set to start the solver

//gui thread relevant variables   ------------------------------------------
extern std::atomic<int> g_programState;
extern std::atomic<bool> g_pause;
extern std::atomic<bool> g_clear_button;
extern std::atomic<bool> g_closeProgram;


//misc variables  ------------------------------------------
extern Model *g_m;
extern std::mutex g_kokkoslock; //for all kokkos operations in threads.

//programm state enum, used for gui-recon communication
enum programStates {
	gui_READY = 0, // we see the start button#
	gui_RUNNING, // We see the stop button, stuff is running!
	gui_PAUSE // Pause State, choose to savem mesh, resume, etc.
};

}


//config variables.

extern float g_td;
extern float g_tc;
extern float g_tn;
extern double g_mergeradius;
extern double g_conditionThres;
extern double g_minArea;
extern double g_reprojection_threshold;
extern int g_nopt;
extern int g_nread;
extern int g_nkeypoints;
extern int g_nLocalGroup;
extern bool g_segment;
extern std::string g_readimagePath;
extern int g_nstart;
extern double g_mincutoff;
extern double g_cutoff;
extern bool g_clientdata; 
extern int g_verbosity;
extern int g_initial_width;
extern int g_initial_height;
extern double g_voxel_length;
extern double g_treint;
extern int g_maxGPUFrames;

//other global variables 

extern int g_warmupProgress;
extern open3d::camera::PinholeCameraIntrinsic g_intrinsic;
extern open3d::camera::PinholeCameraIntrinsic g_lowIntr;
extern open3d::camera::PinholeCameraIntrinsic g_fullIntr;
enum camtyp {typ_kinect, typ_realsense, typ_data, typ_client};
extern camtyp g_camType;

//internal global vars
extern unique_id_counter frame_id_counter;
extern unique_id_counter chunk_id_counter;
extern int g_lowi;
extern int g_lowj;
extern int g_nOrb; //number of bytes in a descriptor of orb
extern int g_nFPFH; 
extern std::mutex g_bufferlock;
extern std::mutex g_protobufflock;

extern const int g_resj; //width
extern const int g_resi; //height

extern int g_nchunk;

//misc vars, mostly here instead of somehwere else for debug purpose.
extern double g_reconIterationTimer;
extern double g_reconChunkItTimer;

