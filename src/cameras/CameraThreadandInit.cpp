#include "core/threadCom.h"
#include "CameraThreadandInit.h"
#include "../cameras/CameraKinect.h"
#include "../utils/coreutil.h"
#include "core/threadCom.h"

using namespace open3d;
using namespace std;

void initialiseCamera() {
	//##########################################initialize Camera ###############################################################
	
	
	
	//Create and open device
	if (g_camType == camtyp::typ_kinect) {
	
		threadCom::g_deviceKinect.close(); //needs to be called to prevent errors on change of loadDataFlag
		bool connected = false;

		while (!connected) {

			if (threadCom::g_closeProgram || threadCom::g_take_dataCam) return;

			try {
				threadCom::g_deviceKinect = threadCom::g_deviceKinect.open(K4A_DEVICE_DEFAULT); //throws exception if there is no device
				//executes next lines if there is no exception
				cout << "Azure Kinect device opened!" << endl;
				connected = true;
			}
			catch (k4a::error e) {
				cerr << e.what() << endl;
				cout << "Retry connecting Kinect camera" << endl;
				this_thread::sleep_for(1000ms);
			}
		}

		//Configuration parameters
		//note: nfov and wfov have different field of views. Binned and unbinned have the same, but with different resolutions and more/less jitter and noise
		k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //note this takes processing power since some stupid mjpeg format is native.
		config.color_resolution = K4A_COLOR_RESOLUTION_1536P; //res is 2048x1536, this is the smallest 4:3 resolution which has max overlap with depth
		config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; //res is 640x576 for K4A_DEPTH_MODE_NFOV_UNBINNED
		config.camera_fps = K4A_FRAMES_PER_SECOND_30;
		config.wired_sync_mode = K4A_WIRED_SYNC_MODE_STANDALONE;
		config.synchronized_images_only = true;

		//Start Cameras
		//needs to be started when reconrun starts, to prevent early disconnect
		threadCom::g_deviceKinect.start_cameras(&config);
		// threadCom::g_deviceKinect.start_imu();

		//Calibration object contains camera specific information and is used for all transformation functions
		threadCom::g_calibrationKinect = threadCom::g_deviceKinect.get_calibration(config.depth_mode, config.color_resolution);
		//printCalibrationColor(calibration);

		//take the color intrinsic since depth img is warped to color. Note: This assumes the camera image is a pinhole camera which is an approximation (todo maybe)
		auto ccc = threadCom::g_calibrationKinect.color_camera_calibration;
		auto param = ccc.intrinsics.parameters.param;
		camera::PinholeCameraIntrinsic intrinsic(ccc.resolution_width, ccc.resolution_height, param.fx, param.fy, param.cx, param.cy);

		//note that the input pictures have to be scalled to this resolution
		g_intrinsic = getScaledIntr(intrinsic, g_resj, g_resi);
		g_intrinsic.width_ = g_resj;
		g_intrinsic.height_ = g_resi;
		g_lowIntr = getScaledIntr(g_intrinsic, g_lowj, g_lowi);
		g_fullIntr = intrinsic;

		threadCom::g_cameraParameterSet = true;

	}

	if (g_camType == camtyp::typ_data) {
		cout << "found data cam \n";
		cout << g_readimagePath + "intrinsic.txt" << endl;
		if (fileExists(g_readimagePath + "intrinsic.txt"))
		{
			cout << "Setting intrinsics from file \n";
			setFromIntrinsicFile(g_readimagePath + "intrinsic.txt");
		}
		else {
			cout << "Warning no intrinsic file found. Taking default intrinsic \n";
			setDefaultIntrinsic();
		}

		threadCom::g_cameraParameterSet = true;

	}
	cout << "Camera Initialized" << endl;
}



// check if there is a camera
void cameraConnectionThreadFunction() {

	// Kokkos::initialize();
	Kokkos::Cuda caminitSpace = SpaceInstance<Kokkos::Cuda>::create();

	while (!threadCom::g_closeProgram)
	{ //Loops as long as there is no device found
		threadCom::g_cameraConnected = false;
		if (!g_clientdata) { // to prevent conflict with local camera if client is localhost
			//kinect
			int kinectDeviceCount = 0;
			try {
				kinectDeviceCount = threadCom::g_deviceKinect.get_installed_count();
			}
			catch (...) {
				cout << "cannot access k4a-device" << endl;
				kinectDeviceCount = 0;
			}

			if (kinectDeviceCount > 0) {
				threadCom::g_cameraConnected = true;
				g_camType = camtyp::typ_kinect;
				//utility::LogInfo("Found {} connected Azure Kinect device(s)\n", k4a_device_get_installed_count());
			}
		}

		//data cam
		if (threadCom::g_take_dataCam) {
			g_camType = camtyp::typ_data;
			threadCom::g_cameraConnected = true;
			//utility::LogInfo("Taking data from harddrive \n");
		}



		if (!threadCom::g_cameraConnected) { //when there is no camera, there cannot be valid cameraparameters
			threadCom::g_cameraParameterSet = false;
		}


		if (threadCom::g_cameraConnected && !threadCom::g_cameraParameterSet) { //initialise on successful connection parameters one time
			initialiseCamera();
			//dont need lock here since grid is never used here
			threadCom::g_m->globalGrid->setIntrinsic(caminitSpace,g_intrinsic);
			cout << "initialized Voxel Grid after receiving camera intrinsic \n";
		}

		//what if cameraParameters are not set after program by cameraInitialise?
		while (threadCom::g_take_dataCam && !threadCom::g_closeProgram) { // prevent locking if user wants to quit program if datacam is true
			this_thread::sleep_for(20ms);
		}

		if(!threadCom::g_closeProgram)
			this_thread::sleep_for(1000ms);
	}
}

