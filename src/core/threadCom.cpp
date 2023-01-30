#include "threadCom.h"
#include "GlobalDefines.h"
namespace threadCom{

//camera variables  ------------------------------------------
std::atomic<bool> g_cameraConnected(false);
std::atomic<bool> g_take_dataCam(false); //take hardrive data
k4a::device g_deviceKinect;//the kinect camera object
k4a::calibration g_calibrationKinect;//the calibration object
std::atomic<bool> g_cameraParameterSet(false);//indicates that a camera is connected and intrinsics set
std::atomic<bool> g_warmupCamera(false);
std::list<std::shared_ptr<Frame>> g_framebuffer; //the camera thread adds frames here for reconrun
std::list<RawFrameDataPointers> g_rawbuffer;

//config variables  ------------------------------------------

//meshing variables  ------------------------------------------
std::atomic<bool> unmeshed_data{false}; //indicates wheter marching cubes has run after integrate or deintegrate.

//tracker variables  ------------------------------------------
std::mutex g_currentposlock; //just for the current position
std::atomic<bool> g_current_slam_finished(false);
std::atomic<bool> g_trackingLost(false);
std::atomic<bool> g_reconThreadFinished(false);

//postprocessing variables  ------------------------------------------
std::atomic<bool> g_postProcessing(false); //sets gui block and pp indicator in gui

//integration variables  ------------------------------------------

//non-linear solver variables  ------------------------------------------
std::mutex g_solverlock; //gpu solver lock
std::atomic<bool> g_solverRunning(false);//indicates that the global solver is running. is set to start the solver

//gui thread relevant variables   ------------------------------------------
std::atomic<int> g_programState(gui_READY);
std::atomic<bool> g_pause(false); //indicates that reconthread should pause
std::atomic<bool> g_clear_button(false); //indicates that reconthread should stop
std::atomic<bool> g_closeProgram(false); //the close command has been issued by the user

//misc variables  ------------------------------------------
Model *g_m;
}

//config variables  ------------------------------------------

float g_td = -1;
float g_tc= -1;
float g_tn= -1;
double g_mergeradius= -1;
double g_conditionThres= -1;
double g_minArea= -1;
double g_reprojection_threshold= -1;
int g_nopt= -1;
int g_nread= -1;
int g_nkeypoints= -1;
int g_nLocalGroup= -1;
bool g_segment=false;
std::string g_readimagePath = "C:/dev/";
int g_nstart= -1;
double g_mincutoff= -1;
double g_cutoff= -1;
bool g_clientdata= -1; 
int g_verbosity= -1;
int g_initial_width= 1440;
int g_initial_height= 800;
double g_voxel_length= -1;
double g_treint= -1;
int g_maxGPUFrames = 1e3;


//other global varialbles

int g_warmupProgress = 0;//indicates the cam warump. Only gets read in main thread.
open3d::camera::PinholeCameraIntrinsic g_intrinsic;
open3d::camera::PinholeCameraIntrinsic g_lowIntr;
open3d::camera::PinholeCameraIntrinsic g_fullIntr;
camtyp g_camType = camtyp::typ_kinect;

//internal global vars
unique_id_counter frame_id_counter;
unique_id_counter chunk_id_counter;
int g_lowi = 60;
int g_lowj = 80;
int g_nOrb = 32;
int g_nFPFH = 33;
std::mutex g_bufferlock;
std::mutex g_protobufflock;

const int g_resi = G_RESI;
const int g_resj = G_RESJ;

int g_nchunk = 11;

//misc vars
double g_reconIterationTimer = 0;
double g_reconChunkItTimer = 0;

