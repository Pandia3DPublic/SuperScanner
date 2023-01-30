#pragma once
#include "Gui/geometry.h"
#include <math.h>


class PandiaView {

public:


	static const double FIELD_OF_VIEW_MAX;
	static const double FIELD_OF_VIEW_MIN;
	static const double FIELD_OF_VIEW_DEFAULT;
	static const double FIELD_OF_VIEW_STEP;

	static const double ZOOM_DEFAULT;
	static const double ZOOM_MIN;
	static const double ZOOM_MAX;
	static const double ZOOM_STEP;

	static const double ROTATION_RADIAN_PER_PIXEL;

	enum ProjectionType {
		Perspective = 0,
		Orthogonal = 1,
	};

public:
	PandiaView(GLFWwindow* window);
	~PandiaView();

	/// Function to set view points
	/// This function obtains OpenGL context and calls OpenGL functions to set
	/// the view point.
	// no it doesnt. it just sets mvp
	void SetViewMatrices( const Eigen::Matrix4d &model_matrix = Eigen::Matrix4d::Identity());


	/// Function to get equivalent pinhole camera parameters (does not support
	/// orthogonal since it is not a real camera view)
	bool ConvertToPinholeCameraParameters(open3d::camera::PinholeCameraParameters &parameters);
	bool ConvertFromPinholeCameraParameters(const open3d::camera::PinholeCameraParameters &parameters);

	void setRealCPos(const Eigen::Matrix4d& extrinsic);


	ProjectionType GetProjectionType() const;
	void SetProjectionParameters();
	virtual void Reset();
	virtual void ChangeFieldOfView(double step);
	virtual void ChangeWindowSize(int width, int height);

	// Function to process scaling
	/// \param scale is the relative distance mouse has scrolled.
	virtual void Scale(double scale);

	// Function to process rotation
	/// \param x and \param y are the distances the mouse cursor has moved.
	/// \param xo and \param yo are the original point coordinate the mouse
	/// cursor started to move from.
	/// Coordinates are measured in screen coordinates relative to the top-left
	/// corner of the window client area.
	virtual void Rotate(double x, double y, double xo = 0.0, double yo = 0.0);

	// Function to process translation
	/// \param x and \param y are the distances the mouse cursor has moved.
	/// \param xo and \param yo are the original point coordinate the mouse
	/// cursor started to move from.
	/// Coordinates are measured in screen coordinates relative to the top-left
	/// corner of the window client area.
	virtual void Translate(double x,
		double y,
		double xo = 0.0,
		double yo = 0.0);

	// Function to process rolling
	/// \param x is the distances the mouse cursor has moved.
	/// Coordinates are measured in screen coordinates relative to the top-left
	/// corner of the window client area.
	virtual void Roll(double x);

	const open3d::geometry::AxisAlignedBoundingBox &GetBoundingBox() const {
		return bounding_box_;
	}

	void ResetBoundingBox() { bounding_box_.Clear(); }

	void FitInGeometry(const open3d::geometry::Geometry &geometry) {
		if (geometry.Dimension() == 3) {
			bounding_box_ += ((const open3d::geometry::Geometry3D &)geometry)
				.GetAxisAlignedBoundingBox();
		}
		SetProjectionParameters();
	}

	void FitInGeometry(GLGeometry& geo) {
		//setBoundingBox(geo.points);
		SetProjectionParameters();
	}

	double GetFieldOfView() const { return field_of_view_; }
	open3d::visualization::gl_util::GLMatrix4f GetMVPMatrix() const { return MVP_matrix_; }
	open3d::visualization::gl_util::GLMatrix4f GetProjectionMatrix() const {
		return projection_matrix_;
	}
	open3d::visualization::gl_util::GLMatrix4f GetViewMatrix() const { return view_matrix_; }
	open3d::visualization::gl_util::GLMatrix4f GetModelMatrix() const { return model_matrix_; }
	open3d::visualization::gl_util::GLVector3f GetEye() const { return eye_.cast<GLfloat>(); }
	open3d::visualization::gl_util::GLVector3f GetLookat() const { return lookat_.cast<GLfloat>(); }
	open3d::visualization::gl_util::GLVector3f GetUp() const { return up_.cast<GLfloat>(); }
	open3d::visualization::gl_util::GLVector3f GetFront() const { return front_.cast<GLfloat>(); }
	open3d::visualization::gl_util::GLVector3f GetRight() const { return right_.cast<GLfloat>(); }
	int GetWindowWidth() const { return window_width_; }
	int GetWindowHeight() const { return window_height_; }
	double GetZNear() const { return z_near_; }
	double GetZFar() const { return z_far_; }

	void SetConstantZNear(double z_near) { constant_z_near_ = z_near; }
	void SetConstantZFar(double z_far) { constant_z_far_ = z_far; }
	void UnsetConstantZNear() { constant_z_near_ = -1; }
	void UnsetConstantZFar() { constant_z_far_ = -1; }
	void setFromIntrinsic (Eigen::Matrix4d& intrinsic);
	void setVirtualIntrinsic();
	void setFOVfromCamera(open3d::camera::PinholeCameraIntrinsic& intrinsic);

	struct MouseControl {
	public:
		bool is_mouse_left_button_down = false;
		bool is_mouse_middle_button_down = false;
		bool is_control_key_down = false;
		bool is_shift_key_down = false;
		bool is_alt_key_down = false;
		bool is_super_key_down = false;
		double mouse_position_x = 0.0;
		double mouse_position_y = 0.0;
	};
	MouseControl mouse_control_;
	void MouseButtonCallback(GLFWwindow *window,int button,int action,int mods);
	void MouseMoveCallback(GLFWwindow *window, double x, double y) ;
	void MouseScrollCallback(GLFWwindow *window, double x, double y);
	void KeyPressCallback(GLFWwindow *window, int key, int scancode, int action, int mods);
	void setBoundingBox(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &points);

	GLFWwindow *window_;
	GLuint framebufferID = 0;
	GLuint texColorBufferID = 0;
	GLuint depthBufferID = 0;
	void genFramebuffer();
	int fbox = 0;
	int fboy = 0;
	void setWindowPos(int x, int y);
	double getFOV_y_FromIntrinsic(open3d::camera::PinholeCameraIntrinsic& intrinsic);
	double getFOV_x_FromIntrinsic(open3d::camera::PinholeCameraIntrinsic& intrinsic);



public:
	int window_width_ = 0;
	int window_height_ = 0;
	open3d::geometry::AxisAlignedBoundingBox bounding_box_;
	Eigen::Vector3d eye_ = Eigen::Vector3d::Zero(); //cam position
	Eigen::Vector3d lookat_ = Eigen::Vector3d::Zero(); //center where the cam looks
	Eigen::Vector3d up_ = Eigen::Vector3d::Zero();
	Eigen::Vector3d front_ = Eigen::Vector3d::Zero(); //vector from center to camera
	Eigen::Vector3d right_ = Eigen::Vector3d::Zero();
	double distance_;
	double field_of_view_;
	double zoom_;
	double view_ratio_;
	double aspect_;
	double z_near_;
	double z_far_;
	double constant_z_near_ = -1;
	double constant_z_far_ = -1;
	double virtual_fov = -1;
	open3d::visualization::gl_util::GLMatrix4f projection_matrix_;
	open3d::visualization::gl_util::GLMatrix4f view_matrix_;
	open3d::visualization::gl_util::GLMatrix4f model_matrix_;
	open3d::visualization::gl_util::GLMatrix4f MVP_matrix_;
	open3d::camera::PinholeCameraIntrinsic virtualIntrinsic;


	bool wireframe =false;
	//open3d::camera::PinholeCameraIntrinsic virtualIntrinsic;
	//open3d::cuda::PinholeCameraIntrinsicCuda virtualIntrinsic_cuda;
	//Eigen::Matrix3d virtualIntrinsic = Eigen::Matrix3d::Zero();
};

