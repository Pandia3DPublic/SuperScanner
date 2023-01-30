#include "geometry.h"
#include "core/threadCom.h"
#include <cassert>
#include "core/integrate.h"


using namespace std;
using namespace open3d;
GLGeometry::GLGeometry(shared_ptr<geometry::TriangleMesh> mesh, int vertex_location, int color_location, int normal_location) {
	vertex_location_ = vertex_location;
	color_location_ = color_location;
	normal_location_ = normal_location;
	geometry_typ = InputTyp::geo_triangleMesh;
	mesh_ = mesh;
	setGeometry();
}

GLGeometry::GLGeometry(std::shared_ptr<GPUVoxelGrid> vg, int vertex_location, int color_location, int normal_location)
{

	vertex_location_ = vertex_location;
	color_location_ = color_location;
	normal_location_ = normal_location;
	geometry_typ = InputTyp::geo_rayCastImage;

	rayPcd = vg->rayPcd; //shallow copy once
	// rayPcdNormals = vg->rayNormals;
	rayPcdColor = vg->rayColorPcd;

	setGeometryRay();
}

GLGeometry::~GLGeometry() {
	if (ndrawElements > 0 && geometry_typ != InputTyp::geo_rayCastImage){ //if ndrawElements is 0 geometry is already deleted in SetGeometry. Prevents second time deletion. It's not a leak
		deleteGeometry();
	}else {
		deleteGeometryRay();
	}
}

void GLGeometry::updateGeometry() {
	if (ndrawElements != 0)
		deleteGeometry();
	setGeometry();
}

void GLGeometry::updateGeometryRay()
{
	deleteGeometryRay();
	setGeometryRay();
}

//assumes that vertex position, colors and normals are called 
//in vec3 vertex_position;
//in vec3 vertex_normal;
//in vec3 vertex_color;
//in shader in that order
void GLGeometry::setGeometry() {

	//save current vao to reset later
	int oldVAO;
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oldVAO);
	if (mesh_->triangles_.size() != 0){
		geometry_typ = InputTyp::geo_triangleMesh;
		//save current vao to reset later
		int oldVAO;
		glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oldVAO);

		getPointsandColorfromMesh(points, colors, mesh_, &normals); //normals pointer just for compatibility with non normals vector meshes

		glGenVertexArrays(1, &VAO);
		glGenBuffers(1, &vertex_position_buffer);
		glGenBuffers(1, &vertex_color_buffer);
		glGenBuffers(1, &normal_buffer);
		glGenBuffers(1, &triange_indice_buffer);

		glBindVertexArray(VAO);

		//position
		glBindBuffer(GL_ARRAY_BUFFER, vertex_position_buffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * points.size(), points.data(), GL_STATIC_DRAW);
		glVertexAttribPointer((GLuint)vertex_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
		glEnableVertexAttribArray((GLuint)vertex_location_);

		//color
		glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * colors.size(), colors.data(), GL_STATIC_DRAW);
		glVertexAttribPointer((GLuint)color_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
		glEnableVertexAttribArray((GLuint)color_location_);

		//normals
		if (normal_location_ != -1)
		{
			glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * normals.size(), normals.data(), GL_STATIC_DRAW);
			glVertexAttribPointer((GLuint)normal_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
			glEnableVertexAttribArray((GLuint)normal_location_);
		}

		//triangles
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, triange_indice_buffer);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Eigen::Vector3i) * mesh_->triangles_.size(), mesh_->triangles_.data(), GL_STATIC_DRAW);
		glBindVertexArray(oldVAO);

		ndrawElements = mesh_->triangles_.size() * 3;

	} else {
		ndrawElements =0;
	}

}

//for raycasted pcd, generate triangle mesh, filter triangles in shader
void GLGeometry::setGeometryRay()
{
	int oldVAO; //to reset to old object at the end of function
	int image_size = rayPcd.extent(1) * rayPcd.extent(0);
	glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oldVAO);

	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &vertex_position_buffer);
	glGenBuffers(1, &vertex_color_buffer);
	glGenBuffers(1, &normal_buffer);
	// glGenBuffers(1, &triange_indice_buffer);
	glBindVertexArray(VAO);

	//do we need lock for raypcd here? todo
	pandia_integration::tsdfLock.lock();
	auto pcd_h = Kokkos::create_mirror_view(rayPcd);
	deep_copy(pcd_h, rayPcd);
	auto colors_h = Kokkos::create_mirror_view(rayPcdColor);
	deep_copy(colors_h, rayPcdColor);
	// auto normals_h = Kokkos::create_mirror_view(rayPcdNormals);
	// deep_copy(normals_h, rayPcdNormals);
	pandia_integration::tsdfLock.unlock();
	//Kokkos::View<Kvec3f **, Kokkos::LayoutRight, Kokkos::HostSpace> pcd_h2("tmp thing", g_resi, g_resj);
	//deep_copy(pcd_h2, pcd_h);

	//position
	glBindBuffer(GL_ARRAY_BUFFER, vertex_position_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Kvec3f) * image_size, pcd_h.data(), GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)vertex_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
	glEnableVertexAttribArray((GLuint)vertex_location_);

	//color
	glBindBuffer(GL_ARRAY_BUFFER, vertex_color_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Kvec3f) * image_size, colors_h.data(), GL_STATIC_DRAW);
	glVertexAttribPointer((GLuint)color_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
	glEnableVertexAttribArray((GLuint)color_location_);

	//normal
	// glBindBuffer(GL_ARRAY_BUFFER, normal_buffer);
	// glBufferData(GL_ARRAY_BUFFER, sizeof(Kvec3f) * image_size, normals_h.data(), GL_STATIC_DRAW);
	// glVertexAttribPointer((GLuint)normal_location_, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
	// glEnableVertexAttribArray((GLuint)normal_location_);

	glBindVertexArray(oldVAO);

	// ndrawElements = triangles.size() *3;
	ndrawElements = image_size;
}

void GLGeometry::deleteGeometry() {


	glBindVertexArray(0);
	glDeleteBuffers(1, &vertex_position_buffer);
	glDeleteBuffers(1, &vertex_color_buffer);
	glDeleteBuffers(1, &triange_indice_buffer);
	glDeleteBuffers(1, &normal_buffer);
	glDeleteVertexArrays(1, &VAO);
	points.clear();
}

void GLGeometry::deleteGeometryRay() {
	glBindVertexArray(0);
	glDeleteBuffers(1, &vertex_position_buffer);
	glDeleteBuffers(1, &vertex_color_buffer);
	glDeleteBuffers(1, &normal_buffer);
	glDeleteVertexArrays(1, &VAO);
}

void GLGeometry::getPointsandColorfromMesh(vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &points, vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &colors, shared_ptr<geometry::TriangleMesh> mesh, vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> *normals)
{

	points.resize(mesh->vertices_.size());
	colors.resize(mesh->vertex_colors_.size());
	if (normals != 0) {
		normals->resize(mesh->vertex_normals_.size());
		for (int i=0; i< mesh->vertex_normals_.size(); i++) {
			(*normals)[i] = mesh->vertex_normals_[i].cast<float>();
		}

		if (mesh->vertices_.size()  != mesh->vertex_normals_.size()) {
			utility::LogWarning("Normal vector not the same length as vertex vector in mesh. \n");
		}
	}

	if (mesh->vertices_.size() != mesh->vertex_colors_.size() ) {
		utility::LogWarning("Color vector not the same length as vertex vector in mesh . \n");
	}

	for (int i=0; i< mesh->vertices_.size(); i++) {
		points[i] = mesh->vertices_[i].cast<float>();
		colors[i] = mesh->vertex_colors_[i].cast<float>();
	}
}

void GLGeometry::bind() {
	glBindVertexArray(VAO);
}

void GLGeometry::draw() {
	bind();
	if(geometry_typ == geo_rayCastImage)
		glDrawArrays(GL_POINTS, 0, ndrawElements); //needs to be glDrawArrays. glDrawElements won't work
	else
		glDrawElements(GL_TRIANGLES, ndrawElements, GL_UNSIGNED_INT, 0); 
}

