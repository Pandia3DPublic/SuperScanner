#include "PandiaView.h"
#include "Gui/guiutil.h"
#include "core/threadCom.h"

// Avoid warning caused by redefinition of APIENTRY macro
// defined also in glfw3.h
#define M_PI       3.14159265358979323846   // pi for stupid window.h definition of min and max
#include <cmath>  // jspark
#include <numeric>

using namespace open3d;
using namespace open3d::visualization;


const double PandiaView::FIELD_OF_VIEW_MAX = 90.0;
const double PandiaView::FIELD_OF_VIEW_MIN = 5.0;
const double PandiaView::FIELD_OF_VIEW_DEFAULT =40.0;
const double PandiaView::FIELD_OF_VIEW_STEP = 5.0;

const double PandiaView::ZOOM_DEFAULT = 0.7;
const double PandiaView::ZOOM_MIN = 0.002;
const double PandiaView::ZOOM_MAX = 5.0;
const double PandiaView::ZOOM_STEP = 0.02;

const double PandiaView::ROTATION_RADIAN_PER_PIXEL = 0.003;

PandiaView::PandiaView(GLFWwindow* window) {

	window_ = window;
	glfwSetWindowUserPointer(window,this);

	auto mouse_button_callback = [](GLFWwindow *window, int button, int action,
		int mods) {
			static_cast<PandiaView *>(glfwGetWindowUserPointer(window))
				->MouseButtonCallback(window, button, action, mods);
	};
	glfwSetMouseButtonCallback(window, mouse_button_callback);


	auto mouse_move_callback = [](GLFWwindow *window, double x, double y) {
		static_cast<PandiaView *>(glfwGetWindowUserPointer(window))
			->MouseMoveCallback(window, x, y);
	};
	glfwSetCursorPosCallback(window, mouse_move_callback);

	auto mouse_scroll_callback = [](GLFWwindow *window, double x, double y) {
		static_cast<PandiaView *>(glfwGetWindowUserPointer(window))
			->MouseScrollCallback(window, x, y);
	};
	glfwSetScrollCallback(window, mouse_scroll_callback);

	auto key_press_callback = [](GLFWwindow *window, int key, int scancode,
		int action, int mods) {
			static_cast<PandiaView *>(glfwGetWindowUserPointer(window))
				->KeyPressCallback(window, key, scancode, action, mods);
	};
	glfwSetKeyCallback(window, key_press_callback);


}

PandiaView::~PandiaView() {
	glDeleteFramebuffers(1, &framebufferID);
}

void PandiaView::SetViewMatrices( const Eigen::Matrix4d  &model_matrix /* = Eigen::Matrix4d::Identity()*/) {
	if (window_height_ <= 0 || window_width_ <= 0) {
		utility::LogWarning(
			"[PandiaView] SetViewPoint() failed because window height and "
			"width are not set.");
		return;
	}
	//glViewport(0, 0, window_width_, window_height_);
	if (GetProjectionType() == ProjectionType::Perspective) {
		// Perspective projection
		z_near_ = constant_z_near_ > 0? constant_z_near_: std::max(0.01 * bounding_box_.GetMaxExtent(),	distance_ -	3.0 * bounding_box_.GetMaxExtent());
		z_far_ = constant_z_far_ > 0? constant_z_far_: distance_ + 3.0 * bounding_box_.GetMaxExtent();	
		projection_matrix_ =gl_util::Perspective(field_of_view_, aspect_, z_near_, z_far_);
	} else {
		// Orthogonal projection
		// We use some black magic to support distance_ in orthogonal view
		z_near_ = constant_z_near_ > 0	? constant_z_near_	: distance_ - 3.0 * bounding_box_.GetMaxExtent();
		z_far_ = constant_z_far_ > 0? constant_z_far_	: distance_ + 3.0 * bounding_box_.GetMaxExtent();
		projection_matrix_ =gl_util::Ortho(-aspect_ * view_ratio_, aspect_ * view_ratio_,
				-view_ratio_, view_ratio_, z_near_, z_far_);
	}
	view_matrix_ = gl_util::LookAt(eye_, lookat_, up_);
	model_matrix_ = model_matrix.cast<GLfloat>();
	MVP_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;
}


//todo why weird twist? Answer: Due to the initial coordinate system. If first frame down is not down all
//subsequent frames have the same issue. Need to use imu to fix this and give first frame in reconstruction
//rotation. Need to change optimization initialization for this too.
//void PandiaView::setRealCPos(Eigen::Matrix4d ex) {
//	Eigen::Vector4d z(0,0,0,1); //camera pos
//	Eigen::Vector4d t(0,0,1,1); //one meter in front of the camera
//	Eigen::Vector4d u(0,1,0,1); // up vector
//	//transform all three
//	z = ex* z;
//	t = ex* t;
//	//u = ex* u;
//
//	front_ = (z-t).block<3,1>(0,0); //form center to camera.
//	lookat_ = t.block<3,1>(0,0);
//	//u = u.normalized();
//	SetProjectionParameters(); //calcs eye from front
//}


void PandiaView::setRealCPos(const Eigen::Matrix4d& ex) {
	Eigen::Vector4d t(0,0,1,1); //one meter in front of the camera
	Eigen::Vector3d u(0,1,0); // up vector
	
	
	t = ex* t;
	up_ = ex.block<3,3>(0,0)* u;

	eye_ = ex.block<3,1>(0,3);
	lookat_ = t.head<3>();

	//for SetProjectionParameters stability
	front_ = eye_ - lookat_; 
	zoom_ =  std::tan(field_of_view_ * 0.5 / 180.0 * M_PI)/ 3.0;
}
////set field of view and aspect
//void PandiaView::setFromIntrinsic(Eigen::Matrix4d& intrinsic) {
//	field_of_view_ = 2 * atan(r/constant_z_near_);
//
//}


PandiaView::ProjectionType PandiaView::GetProjectionType() const {
	if (field_of_view_ == FIELD_OF_VIEW_MIN) {
		return ProjectionType::Orthogonal;
	} else {
		return ProjectionType::Perspective;
	}
}

void PandiaView::Reset() {
	field_of_view_ = FIELD_OF_VIEW_DEFAULT;
	zoom_ = ZOOM_DEFAULT;
	lookat_ = bounding_box_.GetCenter(); //zero vector
	up_ = Eigen::Vector3d(0.0, 1.0, 0.0);
	front_ = Eigen::Vector3d(0.0, 0.0, 1.0); //inverse front of camera
	SetProjectionParameters(); //calc camera pos from this info
}

//mostly calculates the camera position
void PandiaView::SetProjectionParameters() {
	front_ = front_.normalized();
	right_ = up_.cross(front_).normalized();
	if (GetProjectionType() == ProjectionType::Perspective) {
		//view_ratio_ = zoom_ * bounding_box_.GetMaxExtent();
		view_ratio_ = zoom_ * 3;
		distance_ = view_ratio_ / std::tan(field_of_view_ * 0.5 / 180.0 * M_PI);
		eye_ = lookat_ + front_ * distance_;
	} else {
		view_ratio_ = zoom_ * bounding_box_.GetMaxExtent();
		distance_ =	view_ratio_ / std::tan(FIELD_OF_VIEW_STEP * 0.5 / 180.0 * M_PI);
		eye_ = lookat_ + front_ * distance_;
	}
}

void PandiaView::ChangeFieldOfView(double step) {
	field_of_view_ =
		std::max(std::min(field_of_view_ + step * FIELD_OF_VIEW_STEP,
			FIELD_OF_VIEW_MAX),
			FIELD_OF_VIEW_MIN);
	SetProjectionParameters();
}

void PandiaView::ChangeWindowSize(int width, int height) {
	if (framebufferID == 0) {
		window_width_ = width;
		window_height_ = height;
		aspect_ = (double)window_width_ / (double)window_height_;
		//SetProjectionParameters(); //doesnt do anything here
		genFramebuffer();
	}
	if (window_height_ != height || window_width_ != width) { //changed
		glViewport(0,0,width, height);
		//delete the framebuffer
		glDeleteTextures(1,&texColorBufferID);
		glDeleteRenderbuffers(1,&depthBufferID);
		glDeleteFramebuffers(1, &framebufferID);  
		//create new fbo
		window_width_ = width;
		window_height_ = height;
		aspect_ = (double)window_width_ / (double)window_height_;
		//SetProjectionParameters();
		genFramebuffer();

		//virtual camera "callback"
		setVirtualIntrinsic();
	}
}

void PandiaView::Scale(double scale) {
	zoom_ = std::max(std::min(zoom_ + scale * ZOOM_STEP, ZOOM_MAX), ZOOM_MIN);
	SetProjectionParameters();
}

void PandiaView::Rotate(double x,
	double y,
	double xo /* = 0.0*/,
	double yo /* = 0.0*/) {
	// some black magic to do rotation
	double alpha = x * ROTATION_RADIAN_PER_PIXEL;
	double beta = y * ROTATION_RADIAN_PER_PIXEL;
	front_ = (front_ * std::cos(alpha) - right_ * std::sin(alpha)).normalized();
	right_ = up_.cross(front_).normalized();
	front_ = (front_ * std::cos(beta) + up_ * std::sin(beta)).normalized();
	up_ = front_.cross(right_).normalized();
	SetProjectionParameters();
}

void PandiaView::Translate(double x,double y,double xo /* = 0.0*/,double yo /* = 0.0*/) {
	Eigen::Vector3d shift = right_ * (-x) / window_height_ * view_ratio_ * 2.0 +up_ * (-y) / window_height_ * view_ratio_ * 2.0;
	eye_ += shift;
	lookat_ += shift;
	SetProjectionParameters();
}

void PandiaView::Roll(double x) {
	double alpha = x * ROTATION_RADIAN_PER_PIXEL;
	// Rotates up_ vector using Rodrigues' rotation formula.
	// front_ vector is an axis of rotation.
	up_ = up_ * std::cos(alpha) + front_.cross(up_) * std::sin(alpha) +
		front_ * (front_.dot(up_)) * (1.0 - std::cos(alpha));
	up_.normalized();
	SetProjectionParameters();
}

void PandiaView::MouseButtonCallback(GLFWwindow *window,int button,int action,int mods) {
	double x, y;
	glfwGetCursorPos(window, &x, &y);
	x = x -fbox;
	y = y-fboy;
	if (x > 0 && x < window_width_ && y > 0 && y < window_height_){
		mouse_control_.mouse_position_x = x;
		mouse_control_.mouse_position_y = y;
		if (action == GLFW_PRESS) {
			if (button == GLFW_MOUSE_BUTTON_LEFT) {
				mouse_control_.is_mouse_left_button_down = true;
				mouse_control_.is_control_key_down = (mods & GLFW_MOD_CONTROL) != 0;
				mouse_control_.is_shift_key_down = (mods & GLFW_MOD_SHIFT) != 0;
				mouse_control_.is_alt_key_down = (mods & GLFW_MOD_ALT) != 0;
				mouse_control_.is_super_key_down = (mods & GLFW_MOD_SUPER) != 0;
			} else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
				mouse_control_.is_mouse_middle_button_down = true;
			}
		} else {
			mouse_control_.is_mouse_left_button_down = false;
			mouse_control_.is_mouse_middle_button_down = false;
			mouse_control_.is_control_key_down = false;
			mouse_control_.is_shift_key_down = false;
			mouse_control_.is_alt_key_down = false;
			mouse_control_.is_super_key_down = false;
		}
	}
}


void PandiaView::MouseMoveCallback(GLFWwindow *window, double x, double y) {
	x = x -fbox;
	y = y-fboy;
	if (x > 0 && x < window_width_ && y > 0 && y < window_height_){
		if (mouse_control_.is_mouse_left_button_down) {
			if (mouse_control_.is_control_key_down) {
				Translate(x - mouse_control_.mouse_position_x,
					y - mouse_control_.mouse_position_y,
					mouse_control_.mouse_position_x,
					mouse_control_.mouse_position_y);
			} else if (mouse_control_.is_shift_key_down) {
				Roll(x - mouse_control_.mouse_position_x);
			} else {
				Rotate(mouse_control_.mouse_position_x -x,
					mouse_control_.mouse_position_y - y,
					mouse_control_.mouse_position_x,
					mouse_control_.mouse_position_y);
			}
			//is_redraw_required_ = true;
		}
		if (mouse_control_.is_mouse_middle_button_down) {
			Translate(mouse_control_.mouse_position_x -x,
				y - mouse_control_.mouse_position_y,
				mouse_control_.mouse_position_x,
				mouse_control_.mouse_position_y);
			//is_redraw_required_ = true;
		}
		mouse_control_.mouse_position_x = x;
		mouse_control_.mouse_position_y = y;
	} else {
		mouse_control_.is_mouse_left_button_down =false;
		mouse_control_.is_mouse_middle_button_down =false;

	}
}

void PandiaView::MouseScrollCallback(GLFWwindow *window, double x, double y) {
	Scale(y);
	//is_redraw_required_ = true;
}


void PandiaView::KeyPressCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (action == GLFW_RELEASE) {
		return;
	}

	switch (key) {
	case GLFW_KEY_W:
		if (!PandiaGui::fileDialogOpenNow) {
			wireframe = !wireframe;
			utility::LogDebug("[Visualizer] Mesh wireframe rendering {}.\n",wireframe ? "ON" : "OFF");
			if (wireframe) {
				glEnable(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(1.0, 1.0);
			} else {
				glDisable(GL_POLYGON_OFFSET_FILL);
			}
		}
		break;

	case GLFW_KEY_LEFT:
		Translate(-10.0, 0);
		break;
	case GLFW_KEY_RIGHT:
		Translate(10.0, 0);
		break;
	case GLFW_KEY_UP:
		lookat_ -= 0.1 * front_;
		SetProjectionParameters();
		break;
	case GLFW_KEY_DOWN:
		lookat_ += 0.1 * front_;
		SetProjectionParameters();
		break;

	}




}

void PandiaView::setBoundingBox(std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> &points)
{
	open3d::geometry::AxisAlignedBoundingBox box;
	if (points.empty()) {
		box.min_bound_ = Eigen::Vector3d(0.0, 0.0, 0.0);
		box.max_bound_ = Eigen::Vector3d(0.0, 0.0, 0.0);
	} else {
		Eigen::Vector3f tmp = std::accumulate(points.begin(), points.end(), points[0],
			[](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
			return a.array().min(b.array()).matrix();
		});
		box.min_bound_ = tmp.cast<double>();

		tmp = std::accumulate(points.begin(), points.end(), points[0],
		[](const Eigen::Vector3f& a, const Eigen::Vector3f& b) {
			return a.array().max(b.array()).matrix();
		});
		box.max_bound_ =tmp.cast<double>();
	}
	bounding_box_ += box;
}

//generates a framebuffer opengl can draw in
void PandiaView::genFramebuffer() {
	glGenFramebuffers(1, &framebufferID);
	glBindFramebuffer(GL_FRAMEBUFFER, framebufferID);   
	// generate texture
	glGenTextures(1, &texColorBufferID);
	glBindTexture(GL_TEXTURE_2D, texColorBufferID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, window_width_, window_height_, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glBindTexture(GL_TEXTURE_2D, 0);

	// attach it to currently bound framebuffer object
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texColorBufferID, 0);
	//depth and stencil buffer
	glGenRenderbuffers(1, &depthBufferID);
	glBindRenderbuffer(GL_RENDERBUFFER, depthBufferID); 
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, window_width_, window_height_);  
	glBindRenderbuffer(GL_RENDERBUFFER, 0);

	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depthBufferID);

	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		std::cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << std::endl;
	glBindFramebuffer(GL_FRAMEBUFFER, 0);  
}
//for mouse control
void PandiaView::setWindowPos(int x, int y) {
	fbox = x;
	fboy = y;
}


double PandiaView::getFOV_y_FromIntrinsic(open3d::camera::PinholeCameraIntrinsic& intrinsic) {
	return 2 * atan(intrinsic.intrinsic_matrix_(1, 2) / intrinsic.intrinsic_matrix_(1, 1)) * 180.0 / M_PI;
}

double PandiaView::getFOV_x_FromIntrinsic(open3d::camera::PinholeCameraIntrinsic& intrinsic) {

	return 2 * atan(intrinsic.intrinsic_matrix_(0, 2) / intrinsic.intrinsic_matrix_(0, 0)) * 180.0 / M_PI;
}

void PandiaView::setVirtualIntrinsic() {

	virtualIntrinsic.height_ = window_height_;
	virtualIntrinsic.width_ = window_width_;

	auto virtual_height = virtualIntrinsic.height_ / 2;
	auto virtual_width = virtualIntrinsic.width_ / 2;

	Eigen::Matrix3d& v_intrinsic = virtualIntrinsic.intrinsic_matrix_;
	
	auto virtual_fy = virtual_height / (2 * (tan(virtual_fov * M_PI/180.0 / 2)));

	v_intrinsic(0, 2) = virtual_width;
	v_intrinsic(1, 2) = virtual_height;
	v_intrinsic(0, 0) = virtual_fy;
	v_intrinsic(1, 1) = virtual_fy;
	v_intrinsic(2, 2) = 1;

}


void PandiaView::setFOVfromCamera(open3d::camera::PinholeCameraIntrinsic& intrinsic) {
	field_of_view_ = getFOV_y_FromIntrinsic(intrinsic);
	virtual_fov = 1.2 * field_of_view_;
}



//
//
//void PandiaView::setFBOSize(int x, int y) {
//	if (framebufferID == 0) {
//		std::cout << "Warning! checkFBOResize. Generate FBO before changing it's size \n";
//	}
//	if (window_height_ != y || window_width_ != x) { //changed
//		std::cout << "x " << x << " y " << y << " \n";
//		//glViewport(0,0,x,y);
//		//delete the framebuffer
//		glDeleteTextures(1,&texColorBufferID);
//		glDeleteRenderbuffers(1,&depthBufferID);
//		glDeleteFramebuffers(1, &framebufferID);  
//		//create new fbo
//		genFramebuffer();
//	}
//	window_width_ = x;
//	window_height_ = y;
//
//}


//
//void PandiaView::checkFBOResize(GLFWwindow *window,int& renderWidth, int& renderHeight)
//{
//	if (framebufferID == 0) {
//		std::cout << "Warning! checkFBOResize. Generate FBO before changing it's size \n";
//	}
//	if (oldbfosizey != renderHeight || oldwindow_width_ != renderWidth) { //changed
//		std::cout << "changing \n";
//		glViewport(fbox,fboy,renderWidth,renderHeight);
//		//delete the framebuffer
//		glDeleteFramebuffers(1, &framebufferID);  
//		//create new fbo
//		genFramebuffer();
//	}
//	oldbfosizey = renderHeight;
//	oldwindow_width_ = renderWidth;
//
//}
