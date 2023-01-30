#include "CameraKinect.h"
#include "utils/coreutil.h"
#include "utils/matrixutil.h"
#include "CameraThreadandInit.h"
#include "utils/imageutil.h"
#include "core/threadCom.h"
#include <opencv2/imgproc.hpp>
#include "genKps.h"
#include "core/threadCom.h"

using namespace cv;
using namespace std;
using namespace open3d;

static mutex corelock;

void printCalibrationDepth(k4a::calibration &calib)
{
	auto color = calib.color_camera_calibration;
	auto depth = calib.depth_camera_calibration;
	cout << "\n===== Device Calibration information (depth) =====\n";
	cout << "color resolution width: " << color.resolution_width << endl;
	cout << "color resolution height: " << color.resolution_height << endl;
	cout << "depth resolution width: " << depth.resolution_width << endl;
	cout << "depth resolution height: " << depth.resolution_height << endl;
	cout << "principal point x: " << depth.intrinsics.parameters.param.cx << endl;
	cout << "principal point y: " << depth.intrinsics.parameters.param.cy << endl;
	cout << "focal length x: " << depth.intrinsics.parameters.param.fx << endl;
	cout << "focal length y: " << depth.intrinsics.parameters.param.fy << endl;
	cout << "radial distortion coefficients:" << endl;
	cout << "k1: " << depth.intrinsics.parameters.param.k1 << endl;
	cout << "k2: " << depth.intrinsics.parameters.param.k2 << endl;
	cout << "k3: " << depth.intrinsics.parameters.param.k3 << endl;
	cout << "k4: " << depth.intrinsics.parameters.param.k4 << endl;
	cout << "k5: " << depth.intrinsics.parameters.param.k5 << endl;
	cout << "k6: " << depth.intrinsics.parameters.param.k6 << endl;
	cout << "center of distortion in Z=1 plane, x: " << depth.intrinsics.parameters.param.codx << endl;
	cout << "center of distortion in Z=1 plane, y: " << depth.intrinsics.parameters.param.cody << endl;
	cout << "tangential distortion coefficient x: " << depth.intrinsics.parameters.param.p1 << endl;
	cout << "tangential distortion coefficient y: " << depth.intrinsics.parameters.param.p2 << endl;
	cout << "metric radius: " << depth.intrinsics.parameters.param.metric_radius << endl;
	cout << endl;
}
void printCalibrationColor(k4a::calibration &calib)
{
	auto color = calib.color_camera_calibration;
	auto depth = calib.depth_camera_calibration;
	cout << "\n===== Device Calibration information (color) =====\n";
	cout << "color resolution width: " << color.resolution_width << endl;
	cout << "color resolution height: " << color.resolution_height << endl;
	cout << "depth resolution width: " << depth.resolution_width << endl;
	cout << "depth resolution height: " << depth.resolution_height << endl;
	cout << "principal point x: " << color.intrinsics.parameters.param.cx << endl;
	cout << "principal point y: " << color.intrinsics.parameters.param.cy << endl;
	cout << "focal length x: " << color.intrinsics.parameters.param.fx << endl;
	cout << "focal length y: " << color.intrinsics.parameters.param.fy << endl;
	cout << "radial distortion coefficients:" << endl;
	cout << "k1: " << color.intrinsics.parameters.param.k1 << endl;
	cout << "k2: " << color.intrinsics.parameters.param.k2 << endl;
	cout << "k3: " << color.intrinsics.parameters.param.k3 << endl;
	cout << "k4: " << color.intrinsics.parameters.param.k4 << endl;
	cout << "k5: " << color.intrinsics.parameters.param.k5 << endl;
	cout << "k6: " << color.intrinsics.parameters.param.k6 << endl;
	cout << "center of distortion in Z=1 plane, x: " << color.intrinsics.parameters.param.codx << endl;
	cout << "center of distortion in Z=1 plane, y: " << color.intrinsics.parameters.param.cody << endl;
	cout << "tangential distortion coefficient x: " << color.intrinsics.parameters.param.p1 << endl;
	cout << "tangential distortion coefficient y: " << color.intrinsics.parameters.param.p2 << endl;
	cout << "metric radius: " << color.intrinsics.parameters.param.metric_radius << endl;
	cout << endl;
}

//corethread to push raw framesets and imu data to respective buffers
void coreThread(k4a::device *kinect, std::atomic<bool> *stop)
{
	bool success = true;
	while (!(*stop))
	{
		RawFrameDataPointers rawdata;
		rawdata.capture = make_shared<k4a::capture>();
		rawdata.imuVector = make_shared<vector<k4a_imu_sample_t>>();
		//do not check for connection or parameterSet, else you will be stuck in this loop
		while (threadCom::g_pause)
		{
			this_thread::sleep_for(20ms);
		}
		try
		{ //prevents capturing from invalid/disconnected device

			//retrieves capture and imu data from device
			threadCom::g_deviceKinect.get_capture(rawdata.capture.get()); //blocking

			success = true;
		}
		catch (...)
		{
			cout << "connection lost \n";
			success = false;
		}

		//only writes non-empty captures/imus in buffer
		if (success)
		{
			corelock.lock();
			threadCom::g_rawbuffer.push_back(rawdata);
			corelock.unlock();
		}
	}
	corelock.lock();
	threadCom::g_rawbuffer.clear();
	corelock.unlock();
}
//this function takes the heavy load of data preparation
//todo kinect conversion is complicated and there should be some pcd generation in there somewhere.
//		For really high performance this can be used.
//todo check if gpu support is enabled for tranform functiosn of azure sdk
//note: init kamera must be called beforehand
void KinectThread(std::atomic<bool> &stop, std::atomic<bool> &cameraParameterSet)
{

	Kokkos::Cuda kinectSpace = SpaceInstance<Kokkos::Cuda>::create();

	// Kokkos::initialize();
	threadCom::g_warmupCamera = true; //for gui-indicator
	//start record thread, rawbuffer for raw camera frames
	k4a::transformation transform(threadCom::g_calibrationKinect);

	//Warmup for ca. 1 second
	k4a::capture captureWarmup;
	for (int i = 0; i < 60; i++)
	{
		threadCom::g_deviceKinect.get_capture(&captureWarmup); //dropping several frames for auto-exposure
		g_warmupProgress++;
	}
	captureWarmup.reset();
	threadCom::g_warmupCamera = false;
	g_warmupProgress = 0;

	//coreThreads must be startetd after captureWarmup to avoid that first 30 frames get pushed in rawbuffer
	shared_ptr<std::thread> cThread = make_shared<std::thread>(coreThread, &threadCom::g_deviceKinect, &stop); //startet neuen thread und liest bidler ein

	//Camera Loop
	while (!stop)
	{
		while (threadCom::g_rawbuffer.empty() && !stop)
		{
			std::this_thread::sleep_for(20ms);
		}

		if (stop)
		{
			break;
		}

		//Get capture from raw image buffer and from imu-camera data
		corelock.lock();
		auto rawdata = threadCom::g_rawbuffer.front();
		// cout << "################rawbuffer size " << threadCom::g_rawbuffer.size() << endl;
		threadCom::g_rawbuffer.pop_front();
		corelock.unlock();

		//get color and aligned depth image
		//this produces an image that is distorted accoring to the color camera distortion, which should be near to pinhole
		k4a::image color = rawdata.capture->get_color_image();
		k4a::image tmpdepth = rawdata.capture->get_depth_image();
		//takes 3-6ms long, should be gpu supported (untested)
		k4a::image depth = transform.depth_image_to_color_camera(tmpdepth); //depth image now aligned to color!
		//todo: Note the color image is not a perfect pinhole camera. For ideal results there should be one more distortion step for both images
		//To get really fast this should be done on downsized images (optional).
		//To get even faster, skip the opencv images and work directly on k4a images (very optional)

		//############################ transform to scaled kokkos views #################
		// Create OpenCV Mat from Kinect Image
		//opencv stuff takes 1.5ms
		//note this goes out of scope
		Mat cvCol4 = Mat(Size(color.get_width_pixels(), color.get_height_pixels()), CV_8UC4, (void *)color.get_buffer(), Mat::AUTO_STEP);
		Mat cvCol3 = Mat(Size(color.get_width_pixels(), color.get_height_pixels()), CV_8UC3);
		cvtColor(cvCol4, cvCol3, cv::ColorConversionCodes::COLOR_BGRA2RGB); //remove alpha channel
		Mat cvDepth = Mat(Size(depth.get_width_pixels(), depth.get_height_pixels()), CV_16U, (void *)depth.get_buffer(), Mat::AUTO_STEP);
		//note the global intrinsics have to be adjusted to these numbers

		// //for full resolution orb keypoint detection on gpu
		// Mat cvColorFull = Mat(Size(color.get_width_pixels(), color.get_height_pixels()), CV_8UC1);
		// cvtColor(cvCol3, cvColorFull, cv::ColorConversionCodes::COLOR_RGB2GRAY);
		// Kokkos::View<unsigned short **, Kokkos::LayoutRight, Kokkos::HostSpace> depth_full((unsigned short *)cvDepth.data, cvDepth.rows, cvDepth.cols);
		
		resize(cvCol3, cvCol3, Size(g_resj, g_resi));
		resize(cvDepth, cvDepth, Size(g_resj, g_resi), 0, 0, INTER_NEAREST); //no interpol since this will give artifacts for depth images

		//shared_ptr<geometry::Image> color_image = make_shared<geometry::Image>();
		//shared_ptr<geometry::Image> depth_image = make_shared<geometry::Image>();
		Kokkos::View<unsigned char **[3]> color_image("rgb_view", g_resi, g_resj);
		Kokkos::View<unsigned short **> depth_image("depth_view", g_resi, g_resj);

		Kokkos::View<unsigned char **[3], Kokkos::LayoutRight, Kokkos::HostSpace> rgb_orig(cvCol3.data, g_resi, g_resj);
		Kokkos::View<unsigned short **, Kokkos::LayoutRight, Kokkos::HostSpace> depth_orig((unsigned short *)cvDepth.data, g_resi, g_resj);

		auto rgb_h2 = Kokkos::create_mirror_view(color_image);
		Kokkos::deep_copy(rgb_h2, rgb_orig); //this perform layout change. Layoutchange can only happen in same memspace
		Kokkos::deep_copy(color_image, rgb_h2);

		auto depth_h2 = Kokkos::create_mirror_view(depth_image);
		Kokkos::deep_copy(depth_h2, depth_orig); //this perform layout change
		Kokkos::deep_copy(depth_image, depth_h2);

		auto tmp = std::make_shared<Frame>();
		tmp->imuVector = rawdata.imuVector;
		// tmp->timestamp = rawdata.capture->get_color_image().get_device_timestamp().count();
		// cout << "make frame " << kt.seconds() << endl;
		// kt.reset();
		// cout << "nothing " << kt.seconds() << endl;
		// kt.reset();

		// Kokkos::Timer kt;
		generateFrame<Kokkos::Cuda>(color_image, depth_image, tmp, kinectSpace);
		// Kokkos::fence();
		// cout << "full gnerate frame " << kt.seconds() << endl;
		// kt.reset();

		generateOrbKeypoints(tmp, cvCol3, depth_orig);
		// generateOrbKeypoints(tmp, cvColorFull, depth_full);

		g_bufferlock.lock();
		threadCom::g_framebuffer.push_back(tmp);
		g_bufferlock.unlock();
	}
	cThread->join();
}