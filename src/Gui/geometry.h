#pragma once
#include "VoxelGrid/kokkosFunctions.h"
#include "VoxelGrid/GPUVoxelGrid.h"



class GLGeometry {
public:
	//points, colors, indices , vertex location, color location
	GLGeometry(std::shared_ptr<open3d::geometry::TriangleMesh> mesh, int vertex_location, int color_location, int normal_location = -1);
	GLGeometry(std::shared_ptr<GPUVoxelGrid> vg, int vertex_location, int color_location = -1, int normal_location = -1);

	~GLGeometry();

	void updateGeometry();
	void updateGeometryRay();
	
	void setGeometry();
	void setGeometryRay();

	void deleteGeometry();
	void deleteGeometryRay();

	void getPointsandColorfromMesh(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &points, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &colors, std::shared_ptr<open3d::geometry::TriangleMesh> mesh, std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> *normals = 0);
	void getDatafromView(Kokkos::View<Kvec3f **> pcd, Kokkos::View<Kvec3f **> normals);

	void bind();
	void draw();

	// in header as they are needed in multiple functions
	Kokkos::View<Kvec3f **> rayPcd;
	// Kokkos::View<Kvec3f **> rayPcdNormals;
	Kokkos::View<Kvec3f **> rayPcdColor;
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> points;
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> colors;
	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> normals;
	std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>> triangles;
	std::shared_ptr<open3d::geometry::TriangleMesh> mesh_;
	int vertex_location_;
	int color_location_;
	int normal_location_;
	unsigned int VAO, vertex_position_buffer, vertex_color_buffer, triange_indice_buffer, normal_buffer;
	int ndrawElements = 0;
	enum InputTyp {geo_triangleMesh, geo_rayCastImage} geometry_typ;
};