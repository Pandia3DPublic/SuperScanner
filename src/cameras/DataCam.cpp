#include "DataCam.h"
#include "utils/coreutil.h"
#include "utils/imageutil.h"
#include "core/threadCom.h"
#include "genKps.h"
#include <opencv2/imgproc.hpp>

//for hardrive data
template <typename SpaceType>
std::shared_ptr<Frame> getSingleFrame(std::string path, int nstart, SpaceType &space)
{
	using namespace open3d;
	using namespace std;
	using namespace cv;

	std::string filepath_rgb(path + "color/");
	std::string filepath_depth(path + "depth/");

	std::shared_ptr<geometry::Image> rgb_image = std::make_shared<geometry::Image>();
	std::shared_ptr<geometry::Image> depth_image = std::make_shared<geometry::Image>();

	auto tmp = make_shared<Frame>();

	//checks if filepath leads to scenes
	//read scene
	// string filename_scene_depth = filepath_depth + to_string(nstart) + ".png";
	// string filename_scene_rgb = filepath_rgb + to_string(nstart) + ".jpg";

	// read images
	std::string filename_rgb_jpg = filepath_rgb + getPicNumberString(nstart) + ".jpg";
	std::string filename_rgb_png = filepath_rgb + getPicNumberString(nstart) + ".png";
	std::string filename_scene = filepath_rgb + to_string(nstart) + ".jpg";
	//checks if file is .jpg or .png
	auto tmpLevel = utility::GetVerbosityLevel();
	utility::SetVerbosityLevel(utility::VerbosityLevel::Error);
	if (io::ReadImage(filename_rgb_jpg, *rgb_image)) //jpg
	{
		tmp->rgbPath = filename_rgb_jpg;
	}
	else if (io::ReadImage(filename_rgb_png, *rgb_image)) //png
	{
		tmp->rgbPath = filename_rgb_png;
	} else{
		io::ReadImage(filename_scene, *rgb_image); //name with zeros
		tmp->rgbPath = filename_scene;
	}
	std::string filename_depth = filepath_depth + getPicNumberString(nstart) + ".png";
	std::string filename_depth_scene = filepath_depth + to_string(nstart) + ".png";
	if(io::ReadImage(filename_depth, *depth_image)){
		tmp->depthPath = filename_depth;
	} else{
		io::ReadImage(filename_depth_scene, *depth_image);
		tmp->depthPath = filename_depth_scene;
	}

	utility::SetVerbosityLevel(tmpLevel);
	Mat cvColor = Mat(Size(rgb_image->width_, rgb_image->height_), CV_8UC3, (void *)rgb_image->data_.data(), Mat::AUTO_STEP);
	Mat cvDepth = Mat(Size(depth_image->width_, depth_image->height_), CV_16U, (void *)depth_image->data_.data(), Mat::AUTO_STEP);

	// Mat testmat = cvColor.clone();
	// cv::resize(cvColor, testmat, Size(2048, 1536));
	// auto test = make_shared<open3d::geometry::Image>();
	// topen3d(testmat, test);
	// visualization::DrawGeometries({test});

	if (rgb_image->width_ != g_resj || rgb_image->height_ != g_resi)
	{
		cv::resize(cvColor, cvColor, Size(g_resj, g_resi));
	}
	if (depth_image->width_ != g_resj || depth_image->height_ != g_resi)
	{
		cv::resize(cvDepth, cvDepth, Size(g_resj, g_resi), 0, 0, INTER_NEAREST);
	}

	Kokkos::View<unsigned char **[3], Kokkos::LayoutRight, Kokkos::HostSpace> rgb_orig(cvColor.data, g_resi, g_resj);
	Kokkos::View<unsigned short **, Kokkos::LayoutRight, Kokkos::HostSpace> depth_orig((unsigned short *)cvDepth.data, g_resi, g_resj);
	Kokkos::View<unsigned char **[3]> color_image("rgb_view", g_resi, g_resj);
	Kokkos::View<unsigned short **> d_image("depth_view", g_resi, g_resj);

	auto rgb_h2 = Kokkos::create_mirror_view(color_image);

	Kokkos::deep_copy(rgb_h2, rgb_orig); //this perform layout change. Layoutchange can only happen in same memspace
	Kokkos::deep_copy(color_image, rgb_h2);

	auto depth_h2 = Kokkos::create_mirror_view(d_image);

	Kokkos::deep_copy(depth_h2, depth_orig); //this perform layout change
	Kokkos::deep_copy(d_image, depth_h2);

	// auto rgb_low = ResizeColorView(space, color_image, 1536, 2048);
	// auto rgbimg = ViewtoImage(rgb_low);
	// visualization::DrawGeometries({rgbimg});

	generateFrame<Kokkos::Cuda>(color_image, d_image, tmp, space);
	cvtColor(cvColor, cvColor, cv::ColorConversionCodes::COLOR_BGR2RGB);
	//Orb is sensitive to rbg vs bgr!!!!!
	generateOrbKeypoints(tmp, cvColor, depth_h2);

	return tmp;
}

void DataCamThreadFunction(std::atomic<bool> &stop)
{
	using namespace std;
	// Kokkos::initialize();SpaceType
	Kokkos::Cuda datacamSpace = SpaceInstance<Kokkos::Cuda>::create();

	int invTargetFPS = 1000.0 / 30.0; // in ms
	int i = g_nstart;
	while (!stop)
	{
		auto start = std::chrono::high_resolution_clock::now();

		while (threadCom::g_pause)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(20));
		}
		auto tmp = getSingleFrame<Kokkos::Cuda>(g_readimagePath, i, datacamSpace);
		g_bufferlock.lock();
		threadCom::g_framebuffer.push_back(tmp);
		g_bufferlock.unlock();
		i++;
		//no sleep necessary if hardrive access takes forever (e.g. 50ms)
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
		// if (duration.count() < invTargetFPS) // comment in to enable fps cap for camera feeling
		// {
		// 	std::this_thread::sleep_for(std::chrono::milliseconds(invTargetFPS - duration.count())); //to slow down to target fps like 30 fps
		// }
	}
}
