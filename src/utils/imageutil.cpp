#include "imageutil.h"
#include "core/threadCom.h"
#include "VoxelGrid/kokkosFunctions.h"
using namespace open3d;
using namespace std;

KOKKOS_FUNCTION float linear(const float &c1, const float &c2, const float &t) {
	// printf("c1 %f, c2 %f, c2-c1 %f, t %f \n", c1, c2, c2-c1, t);
	return c1 + (c2 - c1) * t;
}

KOKKOS_FUNCTION float doBilinear(const float &topL, const float &topR, const float &botL, const float &botR, const float &tx, const float &ty) {
	return linear(linear(topL, topR, tx), linear(botL, botR, tx), ty);
}

// KOKKOS_FUNCTION float bilinear2(const float &topL, const float &topR, const float &botL, const float &botR, const float &tx, const float &ty) {
// 	return (1 - tx) * (1 - ty) * topL + tx * (1 - ty) * topR + (1 - tx) * ty * botL + tx * ty * botR;
// }

//this is all on gpu
template <typename SpaceType>
Kokkos::View<unsigned char **[3]> ResizeColorView(SpaceType &space, Kokkos::View<unsigned char **[3]> rgb, int resi, int resj, bool bilinear) {
	using namespace Kokkos;

	float resize_factorH = (float)rgb.extent(0) / (float)resi;
	float resize_factorW = (float)rgb.extent(1) / (float)resj;

	View<unsigned char **[3]> out("resize color output view", resi, resj);

	if (bilinear)
	{
		parallel_for(
			"Color Image Resize Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
				// bilinear interpolation
				float giFloat = ((float)i + 0.5) * resize_factorH;
				float gjFloat = ((float)j + 0.5) * resize_factorW;
				int gi = round(giFloat);
				int gj = round(gjFloat);
				// printf("%f, %i, %f, %f\n", giFloat, gi, ((float)i + 0.5), resize_factorH);

				Kvec3f topL(rgb(gi - 1, gj - 1, 0), rgb(gi - 1, gj - 1, 1), rgb(gi - 1, gj - 1, 2));
				Kvec3f topR(rgb(gi - 1, gj, 0), rgb(gi - 1, gj, 1), rgb(gi - 1, gj, 2));
				Kvec3f botL(rgb(gi, gj - 1, 0), rgb(gi, gj - 1, 1), rgb(gi, gj - 1, 2));
				Kvec3f botR(rgb(gi, gj, 0), rgb(gi, gj, 1), rgb(gi, gj, 2));

				out(i, j, 0) = doBilinear(topL.x, topR.x, botL.x, botR.x, gjFloat - gj, giFloat - gi);
				out(i, j, 1) = doBilinear(topL.y, topR.y, botL.y, botR.y, gjFloat - gj, giFloat - gi);
				out(i, j, 2) = doBilinear(topL.z, topR.z, botL.z, botR.z, gjFloat - gj, giFloat - gi);
			});
	}
	else
	{
		parallel_for(
			"Color Image Resize Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
				// no interpolation
				out(i, j, 0) = rgb((int)resize_factorH * i, (int)resize_factorW * j, 0);
				out(i, j, 1) = rgb((int)resize_factorH * i, (int)resize_factorW * j, 1);
				out(i, j, 2) = rgb((int)resize_factorH * i, (int)resize_factorW * j, 2);
			});
	}

	return out;
}

template Kokkos::View<unsigned char **[3]> ResizeColorView<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<unsigned char **[3]> rgb, int resi, int resj, bool bilinear);

template <typename SpaceType>
Kokkos::View<unsigned short **> ResizeDepthView(SpaceType &space, Kokkos::View<unsigned short **> depth, int resi, int resj)
{
	using namespace Kokkos;

	float resize_factorH = (float)depth.extent(0) / (float)resi;
	float resize_factorW = (float)depth.extent(1) / (float)resj;

	// View<unsigned short **> out("resize depth output view", resi, resj);
	View<unsigned short **> out("resize depth output view", resi, resj);

	parallel_for(
		"Depth Image Resize Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			out(i, j) = depth((int)resize_factorH * i, (int)resize_factorW * j);
		});

	return out;
}

template Kokkos::View<unsigned short **> ResizeDepthView<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<unsigned short **> depth, int resi, int resj);

template <typename SpaceType>
Kokkos::View<Kvec3f **> CreatePcdFromDepthView(SpaceType &space, Kokkos::View<unsigned short **> depth, const open3d::camera::PinholeCameraIntrinsic &intr)
{
	using namespace Kokkos;
	int resi = depth.extent(0);
	int resj = depth.extent(1);
	Kokkos::View<Kvec3f **> out("PCD View", resi, resj);
	float cx = intr.intrinsic_matrix_(0, 2);
	float cy = intr.intrinsic_matrix_(1, 2);
	float ci = resi - cy;
	float cj = cx;
	float fx = intr.intrinsic_matrix_(0, 0);
	float fy = intr.intrinsic_matrix_(1, 1);

	// printf("ci %f cj %f fx %f fy %f", ci, cj, fx, fy);

	parallel_for(
		"Depth PCD Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			float d = (float)(depth(i, j)) / 1000; //note: works faster with explicit typecast
			out(i, j) = Kvec3f((j - cj) / fx * d, (i - ci) / fy * d, d);
		});

	return out;
}

template Kokkos::View<Kvec3f **> CreatePcdFromDepthView<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<unsigned short **> depth, const open3d::camera::PinholeCameraIntrinsic &intr);

//simple method that does a lot of bad stuff for edges
template <typename SpaceType>
Kokkos::View<Kvec3f **> CalculateNormalsFromPCDView(SpaceType &space, Kokkos::View<Kvec3f **> pcd)
{
	using namespace Kokkos;
	int resi = pcd.extent(0);
	int resj = pcd.extent(1);
	Kokkos::View<Kvec3f **> out("Normals View", resi, resj);
	parallel_for(
		"Depth Image Normal Init Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			out(i, j) = Kvec3f(0.0, 0.0, 1.0); //this is the default normal vector which is also used by open3d
		});

	parallel_for(
		"Depth Image Normal Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {1, 1}, {resi - 1, resj - 1}), KOKKOS_LAMBDA(int i, int j) {
			if (pcd(i, j).z != 0 && pcd(i+1, j).z != 0 && pcd(i-1, j).z != 0 && pcd(i, j+1).z != 0 && pcd(i, j-1).z != 0)
			{
				// out(i, j) = (pcd(i + 1, j) - pcd(i, j)).cross(pcd(i, j + 1) - pcd(i, j));
				// out(i, j).normalize();

				//this is significantly better
				Kvec3f n1 = (pcd(i + 1, j) - pcd(i, j)).cross(pcd(i, j + 1) - pcd(i, j));
				Kvec3f n2 = (pcd(i, j - 1) - pcd(i, j)).cross(pcd(i + 1, j) - pcd(i, j));
				Kvec3f n3 = (pcd(i, j + 1) - pcd(i, j)).cross(pcd(i - 1, j) - pcd(i, j));
				Kvec3f n4 = (pcd(i - 1, j) - pcd(i, j)).cross(pcd(i, j - 1) - pcd(i, j));

				//note: this can generate normal vectors with z coordinate = 0 like (1,0,0) or (0,1,0)
				out(i, j) = (n1 + n2 + n3 + n4);
				out(i, j).normalize();
				
				// if (out(i,j).z == 0) {
				// 	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				// 	n1.print("n1");
				// 	n2.print("n2");
				// 	n3.print("n3");
				// 	n4.print("n4");
				// 	out(i, j).print("out");
				// }
			}
		});

	return out;
}

template Kokkos::View<Kvec3f **> CalculateNormalsFromPCDView<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<Kvec3f **> pcd);

//between 0 and 1
template <typename SpaceType>
Kokkos::View<float **> CalculateIntensityfromColorView(SpaceType &space, Kokkos::View<unsigned char **[3]> rgb)
{
	using namespace Kokkos;
	int resi = rgb.extent(0);
	int resj = rgb.extent(1);
	Kokkos::View<float **> out("Intensity View", resi, resj);
	parallel_for(
		"Intensity Image Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			out(i, j) = ((float)rgb(i, j, 0) + (float)rgb(i, j, 1) + (float)rgb(i, j, 2)) / 765.0; //255 * 3 
		});

	return out;
}

template Kokkos::View<float **> CalculateIntensityfromColorView<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<unsigned char **[3]> rgb);


template <typename SpaceType>
Kokkos::View<float **> NNSBruteForce(SpaceType &space, Kokkos::View<Kvec3f **> pcd_source, Kokkos::View<Kvec3f **> pcd_target)
{
	using namespace Kokkos;

	// todo implement

	Kokkos::View<float **> out("NNSBruteForce tmp", 1, 1);
	// parallel_for(
	// 	"NNSBruteForce Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
	// 		// Kvec3f diff = pcd_target(i, j) - pcd_source(i, j);
	// 	});

	return out;
}
template Kokkos::View<float **> NNSBruteForce<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<Kvec3f **> pcd_source, Kokkos::View<Kvec3f **> pcd_target);

//########################################################################
void topencv(std::shared_ptr<geometry::Image> &img, cv::Mat &cvimg)
{
	if (img->num_of_channels_ == 3) {
		cvimg = cv::Mat(img->height_, img->width_, CV_8UC3);
		for (int i = 0; i < img->height_; i++)
		{
			uint8_t *pixel = cvimg.ptr<uint8_t>(i); // point to first color in row
			for (int j = 0; j < img->width_; j++)
			{
				*pixel++ = *img->PointerAt<uint8_t>(j, i, 0);
				*pixel++ = *img->PointerAt<uint8_t>(j, i, 1);
				*pixel++ = *img->PointerAt<uint8_t>(j, i, 2);
			}
		}
	}
	else if (img->num_of_channels_ == 1) {
		cvimg = cv::Mat(img->height_, img->width_, CV_16U);
		for (int i = 0; i < img->height_; i++)
		{
			uint16_t *pixel = cvimg.ptr<uint16_t>(i); // point to first pixel in row
			for (int j = 0; j < img->width_; j++)
			{
				*pixel++ = *img->PointerAt<uint16_t>(j, i);
			}
		}
	}
}
void topen3d(cv::Mat &cvimg, std::shared_ptr<open3d::geometry::Image> &img)
{
	if (cvimg.channels() == 3) {
		img->Prepare(cvimg.cols, cvimg.rows, 3, 1);

		#pragma omp parallel for
		for (int y = 0; y < cvimg.rows; y++)
		{
			uint8_t *pixel = cvimg.ptr<uint8_t>(y); // point to first color in row
			for (int x = 0; x < cvimg.cols; x++)
			{
				*img->PointerAt<uint8_t>(x, y, 0) = *pixel++;
				*img->PointerAt<uint8_t>(x, y, 1) = *pixel++;
				*img->PointerAt<uint8_t>(x, y, 2) = *pixel++;
			}
		}
	}
	else if (cvimg.channels() == 1) {
		img->Prepare(cvimg.cols, cvimg.rows, 1, 2);

		#pragma omp parallel for
		for (int y = 0; y < cvimg.rows; y++)
		{
			uint16_t *pixel = cvimg.ptr<uint16_t>(y); // point to first pixel in row
			for (int x = 0; x < cvimg.cols; x++)
			{
				*img->PointerAt<uint16_t>(x, y) = *pixel++;
			}
		}
	}
}
//for color
std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<unsigned char **[3]> img)
{
	std::shared_ptr<open3d::geometry::Image> out = std::make_shared<open3d::geometry::Image>();
	out->Prepare(img.extent(1), img.extent(0), 3, 1);

	auto img_h = create_mirror_view(img);
	deep_copy(img_h, img);

	for (int j = 0; j < img.extent(1); j++)
	{
		for (int i = 0; i < img.extent(0); i++)
		{
			for (int ch = 0; ch < 3; ch++)
			{
				*(out->PointerAt<uint8_t>(j, i, ch)) = img_h(i, j, ch);
			}
		}
	}

	return out;
}

std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<Kvec3f **> img)
{
	std::shared_ptr<open3d::geometry::Image> out = std::make_shared<open3d::geometry::Image>();
	out->Prepare(img.extent(1), img.extent(0), 3, 1);

	auto img_h = create_mirror_view(img);
	deep_copy(img_h, img);

	for (int j = 0; j < img.extent(1); j++)
	{
		for (int i = 0; i < img.extent(0); i++)
		{
			*(out->PointerAt<uint8_t>(j, i, 0)) = img_h(i, j).x * 10;
			*(out->PointerAt<uint8_t>(j, i, 1)) = img_h(i, j).y * 10;
			*(out->PointerAt<uint8_t>(j, i, 2)) = img_h(i, j).z * 10;
		}
	}

	return out;
}

std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<unsigned short **> img)
{
	std::shared_ptr<open3d::geometry::Image> out = std::make_shared<open3d::geometry::Image>();
	out->Prepare(img.extent(1), img.extent(0), 1, 2);

	auto img_h = create_mirror_view(img);
	deep_copy(img_h, img);

	for (int j = 0; j < img.extent(1); j++)
	{
		for (int i = 0; i < img.extent(0); i++)
		{
			*(out->PointerAt<unsigned short>(j, i)) = img_h(i, j);
		}
	}

	return out;
}

std::shared_ptr<open3d::geometry::Image> ViewtoImage(Kokkos::View<float **> img)
{
	std::shared_ptr<open3d::geometry::Image> out = std::make_shared<open3d::geometry::Image>();
	out->Prepare(img.extent(1), img.extent(0), 1, 2);

	auto img_h = create_mirror_view(img);
	deep_copy(img_h, img);

	for (int j = 0; j < img.extent(1); j++)
	{
		for (int i = 0; i < img.extent(0); i++)
		{
			*(out->PointerAt<unsigned short>(j, i)) = (unsigned short)(255*img_h(i, j));
		}
	}

	return out;
}

//For view that are already pcds!
std::shared_ptr<open3d::geometry::PointCloud> ViewtoO3dPCd(Kokkos::View<Kvec3f **> img)
{
	std::shared_ptr<open3d::geometry::PointCloud> out = std::make_shared<open3d::geometry::PointCloud>();

	auto img_h = create_mirror_view(img);
	deep_copy(img_h, img);
	for (int j = 0; j < img.extent(1); j++)
	{
		for (int i = 0; i < img.extent(0); i++)
		{
			if (img_h(i, j).z != 0)
			{
				out->points_.push_back(Eigen::Vector3d(img_h(i, j).x, img_h(i, j).y, img_h(i, j).z));
			}
			// else
			// {
			// 	out->points_.push_back(Eigen::Vector3d::Zero());
			// }
		}
	}
	return out;
}

void AddViewNormalstoO3DPCD(Kokkos::View<Kvec3f **> normals, std::shared_ptr<open3d::geometry::PointCloud> pcd)
{

	auto normals_h = create_mirror_view(normals);
	deep_copy(normals_h, normals);

	for (int j = 0; j < normals_h.extent(1); j++)
	{
		for (int i = 0; i < normals_h.extent(0); i++)
		{
			// need it like this so we don't miss normal vectors like (1,0,0) or (0,1,0). checking only if z!=0 is insufficient
			if (normals_h(i, j).x != 0 || normals_h(i, j).y != 0 || normals_h(i, j).z != 0) 
			{
				pcd->normals_.push_back(Eigen::Vector3d(normals_h(i, j).x, normals_h(i, j).y, normals_h(i, j).z));
			}
		}
	}

	#ifdef ENABLEASSERTIONS
	if (pcd->points_.size() != pcd->normals_.size()) {
		cout << "######## warning: points and normals vector size differs (AddViewNormalstoO3DPCD). will cause serious issues in some open3d functions\n";
		cout << "points size " << pcd->points_.size() << endl;
		cout << "normals size " << pcd->normals_.size() << endl;
		cout << "has normals " << pcd->HasNormals() << endl;
	}
	#endif
}


template <typename SpaceType>
std::shared_ptr<geometry::Image> ColorViewtoImageDownscale(Kokkos::View<unsigned char **[3]> color, int resj, int resi, std::string mode, SpaceType &space)
{
	using namespace Kokkos;

	std::shared_ptr<geometry::Image> out = std::make_shared<geometry::Image>();
	out->Prepare(resj, resi, 3, 1);
	double resize_factorH = (double)color.extent(0) / (double)resi;
	double resize_factorW = (double)color.extent(1) / (double)resj;

	View<unsigned char **[3]> tmp("resize tmp view", resi, resj);

	parallel_for(
		"Color Image Resize Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			tmp(i, j, 0) = color((int)resize_factorH * i, (int)resize_factorW * j, 0);
			tmp(i, j, 1) = color((int)resize_factorH * i, (int)resize_factorW * j, 1);
			tmp(i, j, 2) = color((int)resize_factorH * i, (int)resize_factorW * j, 2);
		});

	auto tmp_h = create_mirror_view(tmp);
	deep_copy(tmp_h, tmp);

	for (int j = 0; j < resj; j++)
	{
		for (int i = 0; i < resi; i++)
		{
			for (int ch = 0; ch < 3; ch++)
			{
				*(out->PointerAt<uint8_t>(j, i, ch)) = tmp_h(i, j, ch);
			}
		}
	}

	return out;
}

template <typename SpaceType>
std::shared_ptr<geometry::Image> DepthViewtoImageDownscale(Kokkos::View<unsigned short **> depth, int resj, int resi, std::string mode, SpaceType &space)
{
	using namespace Kokkos;

	std::shared_ptr<geometry::Image> out = std::make_shared<geometry::Image>();
	out->Prepare(resj, resi, 1, 2);
	double resize_factorH = (double)depth.extent(0) / (double)resi;
	double resize_factorW = (double)depth.extent(1) / (double)resj;

	View<unsigned short **> tmp("resize tmp view", resi, resj);

	parallel_for(
		"Depth Image Resize Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), KOKKOS_LAMBDA(int i, int j) {
			tmp(i, j) = depth((int)resize_factorH * i, (int)resize_factorW * j);
		});

	auto tmp_h = create_mirror_view(tmp);
	deep_copy(tmp_h, tmp);

	for (int j = 0; j < resj; j++)
	{
		for (int i = 0; i < resi; i++)
		{
			*(out->PointerAt<unsigned short>(j, i)) = tmp_h(i, j);
		}
	}

	return out;
}

template std::shared_ptr<geometry::Image> ColorViewtoImageDownscale<Kokkos::Cuda>(Kokkos::View<unsigned char **[3]> color, int resj, int resi, std::string mode, Kokkos::Cuda &space);

template std::shared_ptr<geometry::Image> DepthViewtoImageDownscale<Kokkos::Cuda>(Kokkos::View<unsigned short **> depth, int resj, int resi, std::string mode, Kokkos::Cuda &space);

std::shared_ptr<geometry::Image> resizeImage(std::shared_ptr<geometry::Image> image_ptr, int width, int height, std::string mode)
{

	std::shared_ptr<geometry::Image> out = std::make_shared<geometry::Image>();
	out->Prepare(width, height, image_ptr->num_of_channels_, image_ptr->bytes_per_channel_);
	//auto image = *image_ptr;
	double resize_factorH = (double)image_ptr->height_ / (double)height;
	double resize_factorW = (double)image_ptr->width_ / (double)width;

	//		// std::cout << "resize height: " << resize_height << std::endl;
	//		if (mode == "bilinear") { //todo komische striche noch wegmachen
	//#pragma omp parallel for
	//			for (int x = 0; x < returnimg.width_; x++) {
	//				for (int y = 0; y < returnimg.height_; y++) {
	//					double xres = resize_width * x;
	//					double yres = resize_height * y;
	//					uint8_t bottom_right = *image.PointerAt<uint8_t>(ceil(xres), ceil(yres));
	//					uint8_t bottom_left = *image.PointerAt<uint8_t>(floor(xres), ceil(yres));
	//					uint8_t top_right = *image.PointerAt<uint8_t>(ceil(xres), floor(yres));
	//					uint8_t top_left = *image.PointerAt<uint8_t>(floor(xres), floor(yres));
	//					*returnimg.PointerAt<uint8_t>(x, y) = (ceil(xres) - xres) * (ceil(yres) - yres) * top_left +
	//						(floor(xres) - xres) * (floor(yres) - yres) * bottom_right +
	//						(xres - floor(xres)) * (ceil(yres) - yres) * top_right +
	//						(ceil(xres) - xres) * (yres - floor(yres)) * bottom_left;
	//				}
	//			}
	//		}

	if (image_ptr->bytes_per_channel_ == 1)
	{
		if (mode == "nointerpol")
		{
#pragma omp parallel for
			for (int x = 0; x < out->width_; x++)
			{
				for (int y = 0; y < out->height_; y++)
				{
					for (int ch = 0; ch < image_ptr->num_of_channels_; ch++)
					{
						*(out->PointerAt<uint8_t>(x, y, ch)) = *(image_ptr->PointerAt<uint8_t>(floor(resize_factorW * x), floor(resize_factorH * y), ch));
					}
				}
			}
		}
		else if (mode == "nointerpol")
		{
			//todo
			for (int x = 0; x < out->width_; x++)
			{
				for (int y = 0; y < out->height_; y++)
				{
					for (int ch = 0; ch < image_ptr->num_of_channels_; ch++)
					{
						*(out->PointerAt<uint8_t>(x, y, ch)) = *(image_ptr->PointerAt<uint8_t>(floor(resize_factorW * x), floor(resize_factorH * y), ch));
					}
				}
			}
		}
	}
	else
	{
		if (image_ptr->bytes_per_channel_ == 2)
		{
			if (mode == "nointerpol")
			{
#pragma omp parallel for
				for (int x = 0; x < out->width_; x++)
				{
					for (int y = 0; y < out->height_; y++)
					{
						*(out->PointerAt<uint16_t>(x, y)) = *(image_ptr->PointerAt<uint16_t>(floor(resize_factorW * x), floor(resize_factorH * y)));
					}
				}
			}
		}
	}
	return out;
}
