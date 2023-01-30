#include "visutil.h"
#include "matrixutil.h"
#include "core/threadCom.h"
#include "core/integrate.h"
#include "core/threadCom.h"

using namespace open3d;
using namespace std;

//interal function
std::shared_ptr<geometry::TriangleMesh> createPathMesh(Model &m)
{
	vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transVec;

	for (int i = 0; i < m.chunks.size(); i++)
	{
		transVec.push_back(m.chunks[i]->frames[0]->getFrametoWorldTrans());
	}
	auto wholePath = make_shared<geometry::TriangleMesh>();
	for (int i = 0; i < transVec.size(); i++)
	{
		Eigen::Matrix4d &T = transVec[i];
		auto pathConnector = make_shared<geometry::TriangleMesh>(); //line
		Eigen::Matrix4d lastMatrix = Eigen::Matrix4d::Zero();
		//checks if there are at least 2 camPositions
		if (transVec.size() > 1 && i > 0)
		{
			lastMatrix = transVec[i - 1]; //gets second-to-last Matrix
		}

		//if no 2 camPositions exist there wont be a path added
		if (!lastMatrix.isZero())
		{

			Eigen::Vector3d head = T.block<3, 1>(0, 3);
			Eigen::Vector3d tail = lastMatrix.block<3, 1>(0, 3);

			pathConnector->vertices_.push_back(head); //0
			pathConnector->vertices_.push_back(tail); //1

			pathConnector->triangles_.push_back(Eigen::Vector3i(0, 0, 1));
		}

		//constructs whole camera Path
		*wholePath += *pathConnector;
	}

	//colors Vertices
	for (int j = 0; j < wholePath->vertices_.size(); j++)
	{
		wholePath->vertex_colors_.push_back(Eigen::Vector3d(1.0, 0.1, 0.1));
	}

	wholePath->ComputeVertexNormals();

	return wholePath;
}

shared_ptr<geometry::TriangleMesh> getSingleCamMesh(const Eigen::Matrix4d &T)
{
	auto cameraMesh = make_shared<geometry::TriangleMesh>();
	Eigen::Vector3d origin = T.block<3, 1>(0, 3);
	Eigen::Vector3d a(-0.05, 0.03, 0.1);
	Eigen::Vector3d b(-0.05, -0.03, 0.1);
	Eigen::Vector3d c(0.05, -0.03, 0.1);
	Eigen::Vector3d d(0.05, 0.03, 0.1);

	//sets vertices to cameraposition
	a = (T * Eigen::Vector4d(a(0), a(1), a(2), 1)).block<3, 1>(0, 0);
	b = (T * Eigen::Vector4d(b(0), b(1), b(2), 1)).block<3, 1>(0, 0);
	c = (T * Eigen::Vector4d(c(0), c(1), c(2), 1)).block<3, 1>(0, 0);
	d = (T * Eigen::Vector4d(d(0), d(1), d(2), 1)).block<3, 1>(0, 0);

	cameraMesh->vertices_.push_back(origin); //0
	cameraMesh->vertices_.push_back(a);		 //1
	cameraMesh->vertices_.push_back(b);		 //2
	cameraMesh->vertices_.push_back(c);		 //3
	cameraMesh->vertices_.push_back(d);		 //4

	//constructs camera body
	cameraMesh->triangles_.push_back(Eigen::Vector3i(2, 1, 0));
	cameraMesh->triangles_.push_back(Eigen::Vector3i(3, 2, 0));
	cameraMesh->triangles_.push_back(Eigen::Vector3i(4, 3, 0));
	cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 4, 0));

	//generation "lens" of camera / closing camera-Mesh
	cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 2, 4));
	cameraMesh->triangles_.push_back(Eigen::Vector3i(2, 3, 4));
	cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 3, 4)); //just for generation cross on camera for line-drawing
	cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 2, 3)); //just for generation cross on camera for line-drawing

	return cameraMesh;
}
//also colors them
shared_ptr<geometry::TriangleMesh> createCameraMeshes(vector<Eigen::Matrix4d> &transVec)
{

	auto wholeMesh = make_shared<geometry::TriangleMesh>();
	Eigen::Vector3d col_orange = Eigen::Vector3d(1.0, 0.7, 0.35);
	Eigen::Vector3d col_green = Eigen::Vector3d(0.0, 0.7, 0.35);
	Eigen::Vector3d col_blue = Eigen::Vector3d(1.0, 1.0, 1.0);

	for (int i = 0; i < transVec.size(); i++)
	{
		*wholeMesh += *getSingleCamMesh(transVec[i]);
	}

	//colors Vertices
	double redStep = 0.8 / transVec.size();
	double greenStep = 0.6 / transVec.size();

	//generates gradient color from first to last camPosition
	for (int i = 0; i < wholeMesh->vertices_.size(); i += 5)
	{ //camera mesh has 5 vertices. One cam has one color
		for (int j = i; j < (i + 5); j++)
		{
			wholeMesh->vertex_colors_.push_back(col_blue);
		}
		col_blue(0) -= redStep;
		col_blue(1) -= greenStep;
	}

	return wholeMesh;
}

std::shared_ptr<geometry::TriangleMesh> getCameraPathMesh(Model &m)
{

	shared_ptr<geometry::TriangleMesh> mesh = make_shared<geometry::TriangleMesh>();
	vector<Eigen::Matrix4d> transformations;

	for (int i = 0; i < m.chunks.size(); i++)
	{
		transformations.push_back(m.chunks[i]->frames[0]->getFrametoWorldTrans());
	}

	*mesh += *createCameraMeshes(transformations);
	//*mesh += *createPathMesh(transformations);
	mesh->ComputeVertexNormals();

	return mesh;
}

void visualizeChunk(Chunk &c, bool wordlcoords)
{
	cout << "currently not supported! (visualize Chunk) \n";
	// 	auto f = c.frames;
	// 	std::vector<std::shared_ptr<const geometry::Geometry>> tmp;
	// 	// add frames
	// 	for (int i = 0; i < f.size(); i++) {
	// 		auto pcd = geometry::PointCloud::CreateFromRGBDImage(*(f[i]->rgbd), g_intrinsic);
	// 		if (wordlcoords) {
	// 			pcd->Transform(f[i]->getFrametoWorldTrans());
	// 		}
	// 		tmp.push_back(pcd);

	// 	}

	// 	tmp.push_back(getOrigin());
	// 	visualization::DrawGeometries(tmp);
}

void drawEigen(std::vector<Eigen::Matrix3Xd> vvec)
{
	std::vector<std::shared_ptr<const geometry::Geometry>> tmp;
	auto pcd = std::make_shared<geometry::PointCloud>();
	int count = 0;
	for (auto v : vvec)
	{
		for (int i = 0; i < v.cols(); i++)
		{
			Eigen::Vector3d newpoint(v(0, i), v(1, i), v(2, i));
			pcd->points_.push_back(newpoint);
			if (count == 0)
			{
				pcd->colors_.push_back(Eigen::Vector3d(1.0, 0.0, 0.0)); //red
			}
			else
			{
				pcd->colors_.push_back(Eigen::Vector3d(0.0, 0.0, 1.0)); //blue
			}
		}
		tmp.push_back(pcd);
		count++;
	}
	tmp.push_back(getOrigin());
	visualization::DrawGeometries(tmp);
}

void drawPointVector(std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> v)
{
	std::vector<std::shared_ptr<const geometry::Geometry>> tmp;
	for (int i = 0; i < v.size(); i++)
	{
		auto mesh_ptr = geometry::TriangleMesh::CreateSphere(0.05, 10);
		mesh_ptr->Transform(gettrans(v[i]));
		mesh_ptr->PaintUniformColor(Eigen::Vector3d(255, 0, 0));
		tmp.push_back(mesh_ptr);
	}
	tmp.push_back(getOrigin());
	visualization::DrawGeometries(tmp);
}

void visualizeHomogeniousVector(vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &v)
{
	std::vector<std::shared_ptr<const geometry::Geometry>> vis;
	geometry::PointCloud pcd;
	for (int i = 0; i < v.size(); i++)
	{
		pcd.points_.push_back(v[i].block<3, 1>(0, 0));
	}
	vis.push_back(std::make_shared<geometry::PointCloud>(pcd));
	visualization::DrawGeometries(vis);
}

void visualizecustomskeypoints(std::vector<c_keypoint> &kps)
{
	std::vector<std::shared_ptr<const geometry::Geometry>> tmp;
	geometry::PointCloud pcd;
	for (int i = 0; i < kps.size(); i++)
	{
		pcd.points_.push_back(kps[i].p.block<3, 1>(0, 0));
	}
	tmp.push_back(std::make_shared<geometry::PointCloud>(pcd));
	visualization::DrawGeometries(tmp);
}

//draws all pcds with keypoints and lines between them
void visualizecurrentMatches(shared_ptr<Chunk> c)
{
	double r = 2 * 3.14159265359 / (double)(c->frames.size() + 1);
	std::vector<std::shared_ptr<const geometry::Geometry>> vis;
	for (int i = 0; i < c->frames.size(); i++)
	{
		c->frames[i]->MovetoGPU();
		auto rgb = ViewtoImage(c->frames[i]->rgb);
		auto depth = ViewtoImage(c->frames[i]->depth);
		auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(*rgb, *depth);
		auto tmp = *geometry::PointCloud::CreateFromRGBDImage(*rgbd, g_intrinsic);
		Eigen::Matrix4d tmp2 = getRy(r * i) * gettrans(Eigen::Vector3d(0, 0, 1.5));
		tmp.Transform(tmp2);
		vis.push_back(make_shared<geometry::PointCloud>(tmp));
	}

	std::shared_ptr<geometry::LineSet> ls = std::make_shared<geometry::LineSet>();
	for (int i = 0; i < c->frames.size() - 1; i++)
	{
		for (int j = i + 1; j < c->frames.size(); j++)
		{
			auto &matches = c->pairTransforms(c->frames[i], c->frames[j]).filteredmatches;
			for (int k = 0; k < matches.size(); k++)
			{
				auto mesh_ptr = geometry::TriangleMesh::CreateSphere(0.02, 10);
				Eigen::Matrix4d t = getRy(r * i) * gettrans(Eigen::Vector3d(0, 0, 1.5)) * gettrans(matches[k].p1);
				cout << t << endl;
				mesh_ptr->Transform(t);
				mesh_ptr->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
				vis.push_back(mesh_ptr);
				auto mesh_ptr2 = geometry::TriangleMesh::CreateSphere(0.02, 10);
				Eigen::Matrix4d t2 = getRy(r * j) * gettrans(Eigen::Vector3d(0, 0, 1.5)) * gettrans(matches[k].p2);
				mesh_ptr2->Transform(t2);
				mesh_ptr2->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
				vis.push_back(mesh_ptr2);
				ls->points_.push_back((t2 * Eigen::Vector4d(0, 0, 0, 1)).block<3, 1>(0, 0));
				ls->points_.push_back((t * Eigen::Vector4d(0, 0, 0, 1)).block<3, 1>(0, 0));
				ls->lines_.push_back(Eigen::Vector2i(ls->points_.size() - 1, ls->points_.size() - 2));
			}
		}
	}
	vis.push_back(ls);
	vis.push_back(getOrigin());
	visualization::DrawGeometries(vis);
}

//draws all pcds with keypoints and lines between them
//if raw is set to true the unfiltered matches are plotted
void visualizecurrentMatches(Model &m, bool raw /* =false */)
{
	//pcds
	std::vector<std::shared_ptr<const geometry::Geometry>> vis;
	for (int j = 0; j < m.chunks.size(); j++)
	{
		if (m.chunks[j]->frames.size() == 11)
		{
			auto &c = m.chunks[j];
			geometry::PointCloud tmppcd;
			for (int i = 0; i < c->frames.size(); i++)
			{
				if (c->frames[i]->duplicate == false)
				{ //duplicate frame causes runtime error createfromrgbdimage in second chunk
					auto tmprgbd = geometry::RGBDImage::CreateFromColorAndDepth(*ViewtoImage(c->frames[i]->rgb), *ViewtoImage(c->frames[i]->depth), 1000.0, 3.0, false);
					auto tmp = *geometry::PointCloud::CreateFromRGBDImage(*tmprgbd, g_intrinsic);
					// visualization::DrawGeometries({tmprgbd});
					// visualization::DrawGeometries({make_shared<geometry::PointCloud>(tmp)});
					tmp.Transform(c->frames[i]->chunktransform);
					tmppcd = tmppcd + tmp;
				}
			}
			tmppcd.VoxelDownSample(0.005);
			Eigen::Matrix4d tmp2 = getRy(0.5 * j) * gettrans(Eigen::Vector3d(0, 0, 1.5));
			tmppcd.Transform(tmp2);
			vis.push_back(make_shared<geometry::PointCloud>(tmppcd));
		}
	}
	//invalids
	cout << "blubber \n";
	for (int j = 0; j < m.invalidChunks.size(); j++)
	{
		if (m.chunks[j]->frames.size() == 11)
		{
			auto &c = m.invalidChunks[j];
			geometry::PointCloud tmppcd;
			for (int i = 0; i < c->frames.size(); i++)
			{
				if (c->frames[i]->duplicate == false)
				{
					auto tmprgbd = geometry::RGBDImage::CreateFromColorAndDepth(*ViewtoImage(c->frames[i]->rgb), *ViewtoImage(c->frames[i]->depth));
					auto tmp = *geometry::PointCloud::CreateFromRGBDImage(*tmprgbd, g_intrinsic);
					tmp.Transform(c->frames[i]->chunktransform);
					tmppcd = tmppcd + tmp;
				}
			}
			tmppcd.VoxelDownSample(0.005);
			Eigen::Matrix4d tmp2 = getRy(0.5 * (j + m.chunks.size())) * gettrans(Eigen::Vector3d(0, 0, 1.5));
			tmppcd.Transform(tmp2);
			vis.push_back(make_shared<geometry::PointCloud>(tmppcd));
		}
	}

	std::shared_ptr<geometry::LineSet> ls = std::make_shared<geometry::LineSet>();
	int n = m.chunks.size();
	if (raw)
	{
		n += m.invalidChunks.size();
	}
	for (int i = 0; i < n - 1; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			auto &matches = m.pairTransforms(m.chunks[i], m.chunks[j]).filteredmatches;
			if (raw)
			{
				// matches = m.rawmatches(i, j);
			}
			for (int k = 0; k < matches.size(); k++)
			{
				auto mesh_ptr = geometry::TriangleMesh::CreateSphere(0.02, 10);
				Eigen::Matrix4d t = getRy(0.5 * i) * gettrans(Eigen::Vector3d(0, 0, 1.5)) * gettrans(matches[k].p1);
				mesh_ptr->Transform(t);
				mesh_ptr->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
				vis.push_back(mesh_ptr);
				auto mesh_ptr2 = geometry::TriangleMesh::CreateSphere(0.02, 10);
				Eigen::Matrix4d t2 = getRy(0.5 * j) * gettrans(Eigen::Vector3d(0, 0, 1.5)) * gettrans(matches[k].p2);
				mesh_ptr2->Transform(t2);
				mesh_ptr2->PaintUniformColor(Eigen::Vector3d(1, 0, 0));
				vis.push_back(mesh_ptr2);
				ls->points_.push_back((t2 * Eigen::Vector4d(0, 0, 0, 1)).block<3, 1>(0, 0));
				ls->points_.push_back((t * Eigen::Vector4d(0, 0, 0, 1)).block<3, 1>(0, 0));
				ls->lines_.push_back(Eigen::Vector2i(ls->points_.size() - 1, ls->points_.size() - 2));
			}
		}
	}
	vis.push_back(ls);
	vis.push_back(getOrigin());
	visualization::DrawGeometries(vis);
}

// x_nearest are y points closest to x
void visualizeNearestPoints(Kokkos::View<Kvec3f **> x_nearest, Kokkos::View<Kvec3f **> x, Kokkos::View<Kvec3f **> y, int sampling)
{
	// kokkos pcd to host
	// host to o3d pcd
	// draw lines between x and x_nearest

	using namespace Kokkos;
	auto x_nearest_h = create_mirror_view(x_nearest);
	auto x_h = create_mirror_view(x);
	auto y_h = create_mirror_view(y);
	deep_copy(x_nearest_h, x_nearest);
	deep_copy(x_h, x);
	deep_copy(y_h, y);

	shared_ptr<open3d::geometry::PointCloud> x_pcd = make_shared<open3d::geometry::PointCloud>();
	shared_ptr<open3d::geometry::PointCloud> y_pcd = make_shared<open3d::geometry::PointCloud>();
	shared_ptr<open3d::geometry::PointCloud> x_nearest_pcd = make_shared<open3d::geometry::PointCloud>();
	shared_ptr<open3d::geometry::LineSet> ls = make_shared<open3d::geometry::LineSet>();

	int count = 0;
	for (int i = 0; i < x_h.extent(0); i++)
	{
		for (int j = 0; j < x_h.extent(1); j++)
		{
			count++;
			if (count == sampling)
			{

				if (x_h(i, j).z != 0)
				{
					x_pcd->points_.push_back(Eigen::Vector3d(x_h(i, j).x, x_h(i, j).y, x_h(i, j).z));
				}
				count = 0;
			}
		}
	}
	count = 0;
	for (int i = 0; i < y_h.extent(0); i++)
	{
		for (int j = 0; j < y_h.extent(1); j++)
		{
			count++;

			if (count == sampling)
			{
				if (y_h(i, j).z != 0)
				{
					y_pcd->points_.push_back(Eigen::Vector3d(y_h(i, j).x, y_h(i, j).y, y_h(i, j).z));
				}
				count = 0;
			}
		}
	}
	count = 0;
	for (int i = 0; i < y_h.extent(0); i++)
	{
		for (int j = 0; j < y_h.extent(1); j++)
		{
			count++;

			if (count == sampling)
			{

				if (y_h(i, j).z != 0)
				{
					x_nearest_pcd->points_.push_back(Eigen::Vector3d(x_nearest_h(i, j).x, x_nearest_h(i, j).y, x_nearest_h(i, j).z));
				}
				count = 0;
			}
		}
	}

	count = 0;
	for (int i = 0; i < x_nearest_h.extent(0); i++)
	{
		for (int j = 0; j < x_nearest_h.extent(1); j++)
		{
			count++;
			if (count == sampling)
			{
				if (x_nearest_h(i, j).z != 0)
				{
					Eigen::Vector3d X_nearest = Eigen::Vector3d(x_nearest_h(i, j).x, x_nearest_h(i, j).y, x_nearest_h(i, j).z);
					Eigen::Vector3d X = Eigen::Vector3d(x_h(i, j).x, x_h(i, j).y, x_h(i, j).z);
					ls->points_.push_back(X_nearest);
					ls->points_.push_back(X);
					ls->lines_.push_back(Eigen::Vector2i(ls->points_.size() - 1, ls->points_.size() - 2));
				}
				count = 0;
			}
		}
	}
	x_pcd->PaintUniformColor(Eigen::Vector3d(1.0, 0.0, 0.0));
	y_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 0.0, 1.0));
	x_nearest_pcd->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
	// Eigen::Matrix4d trans = Eigen::Matrix4d::Identity();
	// trans.block<3, 1>(0, 3) = Eigen::Vector3d(0.0005, 0, 0);
	// x_nearest_pcd->Transform(trans);

	// visualization::DrawGeometries({x_pcd, y_pcd, x_nearest_pcd, ls});
	visualization::DrawGeometries({x_pcd, x_nearest_pcd, ls});
}

std::shared_ptr<geometry::LineSet> getCamera(Eigen::Matrix4d &t, Eigen::Vector3d color)
{
	auto ls = std::make_shared<geometry::LineSet>();
	Eigen::Vector3d zero(0.0, 0.0, 0.0);
	Eigen::Vector3d a(-0.05, 0.03, 0.1);
	Eigen::Vector3d b(-0.05, -0.03, 0.1);
	Eigen::Vector3d c(0.05, -0.03, 0.1);
	Eigen::Vector3d d(0.05, 0.03, 0.1);
	glEnable(GL_LINE_WIDTH);
	glLineWidth(100.f);
	zero = (t * Eigen::Vector4d(zero(0), zero(1), zero(2), 1)).block<3, 1>(0, 0);
	a = (t * Eigen::Vector4d(a(0), a(1), a(2), 1)).block<3, 1>(0, 0);
	b = (t * Eigen::Vector4d(b(0), b(1), b(2), 1)).block<3, 1>(0, 0);
	c = (t * Eigen::Vector4d(c(0), c(1), c(2), 1)).block<3, 1>(0, 0);
	d = (t * Eigen::Vector4d(d(0), d(1), d(2), 1)).block<3, 1>(0, 0);

	ls->points_.push_back(zero);
	ls->points_.push_back(a);
	ls->points_.push_back(b);
	ls->points_.push_back(c);
	ls->points_.push_back(d);

	ls->lines_.push_back(Eigen::Vector2i(0, 1));
	ls->lines_.push_back(Eigen::Vector2i(0, 2));
	ls->lines_.push_back(Eigen::Vector2i(0, 3));
	ls->lines_.push_back(Eigen::Vector2i(0, 4));

	ls->lines_.push_back(Eigen::Vector2i(1, 2));
	ls->lines_.push_back(Eigen::Vector2i(1, 4));
	ls->lines_.push_back(Eigen::Vector2i(3, 2));
	ls->lines_.push_back(Eigen::Vector2i(3, 4));

	for (int i = 0; i < 8; i++)
	{
		ls->colors_.push_back(color);
	}

	return ls;
}
std::shared_ptr<geometry::LineSet> getCameraPath(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v, Eigen::Vector3d color)
{

	auto ls = std::make_shared<geometry::LineSet>();
	for (int i = 0; i < v.size(); i++)
	{
		*ls += *getCamera(v[i], color);
	}
	Eigen::Vector3d red(1.0, 0.0, 0.0);
	for (int i = 0; i < v.size() - 1; i++)
	{
		ls->lines_.push_back(Eigen::Vector2i(5 * i, 5 * (i + 1)));
		ls->colors_.push_back(red);
	}
	return ls;
}

std::shared_ptr<geometry::LineSet> getOrigin()
{
	auto ls = std::make_shared<geometry::LineSet>();
	Eigen::Vector3d zero(0.0, 0.0, 0.0);
	Eigen::Vector3d x(1.0, 0.0, 0.0);
	Eigen::Vector3d y(0.0, 1.0, 0.0);
	Eigen::Vector3d z(0.0, 0.0, 1.0);

	Eigen::Vector3d xcolor(1.0, 0.0, 0.0);
	Eigen::Vector3d ycolor(0.0, 1.0, 0.0);
	Eigen::Vector3d zcolor(0.0, 0.0, 1.0);

	ls->points_.push_back(zero);
	ls->points_.push_back(x);
	ls->points_.push_back(y);
	ls->points_.push_back(z);
	ls->lines_.push_back(Eigen::Vector2i(0, 1));
	ls->lines_.push_back(Eigen::Vector2i(0, 2));
	ls->lines_.push_back(Eigen::Vector2i(0, 3));
	ls->colors_.push_back(xcolor);
	ls->colors_.push_back(ycolor);
	ls->colors_.push_back(zcolor);

	return ls;
}

std::shared_ptr<geometry::LineSet> getFrustumLineSet(Frustum &fr, const Eigen::Vector3d &color)
{
	auto ls = std::make_shared<geometry::LineSet>();

	for (auto &p : fr.corners)
	{
		ls->points_.push_back(p);
	}

	//cc again starting with back half
	ls->lines_.push_back(Eigen::Vector2i(0, 1));
	ls->lines_.push_back(Eigen::Vector2i(1, 2));
	ls->lines_.push_back(Eigen::Vector2i(2, 3));
	ls->lines_.push_back(Eigen::Vector2i(3, 0));

	//sides
	ls->lines_.push_back(Eigen::Vector2i(0, 4));
	ls->lines_.push_back(Eigen::Vector2i(1, 5));
	ls->lines_.push_back(Eigen::Vector2i(2, 6));
	ls->lines_.push_back(Eigen::Vector2i(3, 7));

	//front
	ls->lines_.push_back(Eigen::Vector2i(4, 5));
	ls->lines_.push_back(Eigen::Vector2i(5, 6));
	ls->lines_.push_back(Eigen::Vector2i(6, 7));
	ls->lines_.push_back(Eigen::Vector2i(7, 4));

	for (int i = 0; i < 12; i++)
	{
		ls->colors_.push_back(color);
	}

	return ls;
}

std::shared_ptr<geometry::LineSet> getvisFrusti(Model &m, Frustum &corefrustum)
{
	auto ls = std::make_shared<geometry::LineSet>();
	vector<Frustum> frusti;
	for (auto &c : m.chunks)
	{
		frusti.push_back(*c->frustum);
	}
	*ls += *getFrustumLineSet(corefrustum, Eigen::Vector3d(0, 0, 1)); //blue

	for (int i = 0; i < frusti.size(); i++)
	{
		//if (corefrustum.intersect(frusti[i])) {
		if (corefrustum.inLocalGroup(frusti[i]))
		{
			*ls += *getFrustumLineSet(frusti[i], Eigen::Vector3d(0, 1, 0)); //green
		}
		else
		{
			*ls += *getFrustumLineSet(frusti[i], Eigen::Vector3d(1, 0, 0)); //red
			cout << "chunk " << i << " not in local group vis\n";
		}
	}

	return ls;
}

//interal function
shared_ptr<geometry::TriangleMesh> createCameraMesh(vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &transVec)
{

	auto wholeMesh = make_shared<geometry::TriangleMesh>();
	Eigen::Vector3d col_orange = Eigen::Vector3d(1.0, 0.7, 0.35);
	Eigen::Vector3d col_green = Eigen::Vector3d(0.0, 0.7, 0.35);
	Eigen::Vector3d col_blue = Eigen::Vector3d(1.0, 1.0, 1.0);

	for (int i = 0; i < transVec.size(); i++)
	{

		Eigen::Matrix4d t = transVec[i];
		auto cameraMesh = make_shared<geometry::TriangleMesh>();
		Eigen::Vector3d zero(0.0, 0.0, 0.0);
		Eigen::Vector3d a(-0.05, 0.03, 0.1);
		Eigen::Vector3d b(-0.05, -0.03, 0.1);
		Eigen::Vector3d c(0.05, -0.03, 0.1);
		Eigen::Vector3d d(0.05, 0.03, 0.1);

		//sets vertices to cameraposition
		zero = (t * Eigen::Vector4d(zero(0), zero(1), zero(2), 1)).block<3, 1>(0, 0);
		a = (t * Eigen::Vector4d(a(0), a(1), a(2), 1)).block<3, 1>(0, 0);
		b = (t * Eigen::Vector4d(b(0), b(1), b(2), 1)).block<3, 1>(0, 0);
		c = (t * Eigen::Vector4d(c(0), c(1), c(2), 1)).block<3, 1>(0, 0);
		d = (t * Eigen::Vector4d(d(0), d(1), d(2), 1)).block<3, 1>(0, 0);

		cameraMesh->vertices_.push_back(zero); //0
		cameraMesh->vertices_.push_back(a);	   //1
		cameraMesh->vertices_.push_back(b);	   //2
		cameraMesh->vertices_.push_back(c);	   //3
		cameraMesh->vertices_.push_back(d);	   //4

		//constructs camera body
		cameraMesh->triangles_.push_back(Eigen::Vector3i(2, 1, 0));
		cameraMesh->triangles_.push_back(Eigen::Vector3i(3, 2, 0));
		cameraMesh->triangles_.push_back(Eigen::Vector3i(4, 3, 0));
		cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 4, 0));

		//generation "lens" of camera / closing camera-Mesh
		cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 2, 4));
		cameraMesh->triangles_.push_back(Eigen::Vector3i(2, 3, 4));
		cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 3, 4)); //just for generation cross on camera for line-drawing
		cameraMesh->triangles_.push_back(Eigen::Vector3i(1, 2, 3)); //just for generation cross on camera for line-drawing

		*wholeMesh += *cameraMesh;
	}

	//colors Vertices
	auto redStep = 0.8 / transVec.size();
	auto greenStep = 0.6 / transVec.size();

	//generates gradient color from first to last camPosition
	for (int i = 0; i < wholeMesh->vertices_.size(); i += 5)
	{

		for (int j = i; j < (i + 5); j++)
		{
			wholeMesh->vertex_colors_.push_back(col_blue);
		}

		col_blue(0) -= redStep;
		col_blue(1) -= greenStep;
	}

	//first camera Position
	for (int i = 0; i < 5; i++)
	{
		//wholeMesh->vertex_colors_[i] = col_green;
		wholeMesh->vertex_colors_[i] = Eigen::Vector3d(0.0, 0.0, 0.0);
	}

	//last camera Position
	auto lastCamVertices = wholeMesh->vertex_colors_.size() - 5;
	for (int i = lastCamVertices; i < wholeMesh->vertex_colors_.size(); i++)
	{
		//wholeMesh->vertex_colors_[i] = col_orange;
		wholeMesh->vertex_colors_[i] = Eigen::Vector3d(0.0, 0.0, 0.0);
	}

	wholeMesh->ComputeTriangleNormals();

	return wholeMesh;
}

//interal function
shared_ptr<geometry::TriangleMesh> createPathMesh(vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &transVec)
{

	auto wholePath = make_shared<geometry::TriangleMesh>();

	for (int i = 0; i < transVec.size(); i++)
	{
		Eigen::Matrix4d t = transVec[i];

		auto pathConnector = make_shared<geometry::TriangleMesh>();
		Eigen::Matrix4d lastMatrix = Eigen::Matrix4d::Zero();

		//checks if there are at least 2 camPositions
		if (transVec.size() > 1 && i > 0)
		{
			lastMatrix = transVec[i - 1]; //gets second-to-last Matrix
		}

		//if no 2 camPositions exist there wont be a path added
		if (!lastMatrix.isZero())
		{

			Eigen::Vector3d head(t(0, 3), t(1, 3), t(2, 3));
			Eigen::Vector3d tail(lastMatrix.block<3, 1>(0, 3));

			pathConnector->vertices_.push_back(head); //0
			pathConnector->vertices_.push_back(tail); //1

			pathConnector->triangles_.push_back(Eigen::Vector3i(0, 0, 1));
		}

		//constructs whole camera Path
		*wholePath += *pathConnector;
	}

	//colors Vertices
	for (int j = 0; j < wholePath->vertices_.size(); j++)
	{
		wholePath->vertex_colors_.push_back(Eigen::Vector3d(1.0, 0.1, 0.1));
	}

	return wholePath;
}

std::shared_ptr<geometry::TriangleMesh> getCameraPathMesh(Model &m, int &divider)
{

	shared_ptr<geometry::TriangleMesh> mesh = make_shared<geometry::TriangleMesh>();

	vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> transformations;

	for (int i = 0; i < m.chunks.size(); i++)
	{
		for (int j = 0; j < 10; j++)
		{

			if (j == 0)
			{ //first frame of every chunk will be drawn as cameraPathMesh

				Eigen::Matrix4d matrix = m.chunks[i]->frames[j]->getFrametoWorldTrans();
				transformations.push_back(matrix);
			}
		}
	}

	*mesh += *createCameraMesh(transformations);
	divider = mesh->vertices_.size();
	*mesh += *createPathMesh(transformations);

	mesh->ComputeVertexNormals();

	return mesh;
}
