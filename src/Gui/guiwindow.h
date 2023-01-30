#pragma once
#include "Gui/guiutil.h"

//This shows the majority of the PandiaGui-windows and its functionality

namespace PandiaGui {

	// initialises docking for whole GLFW-window and sets a menubar on upper window-border
	// menubar contains options for language, shader, developer-window-tools, etc.
	void window_Menubar(PandiaView& v);

	// shows small window in OpenGL-window if reconrun has started
	// indicates progress of warmUp-phase of camera
	void window_WarmUp();

	// shows info text in OpenGL-window, if no camera is connected for live-mode
	void window_CameraConnection();

	// shows window in OpenGL-window, if postprocessing has started
	// indicates progress of postprocessing algorithms for user
	// voxel-length changes can be canceled by button
	void window_PostProcessing();

	// shows small window in corner of OpenGL-window if camerapath is shown
	// indicates colored progression of camera positions
	void window_CameraLegend(Model& m);

	// shows currently reconstructed model and tracking status
	void window_OpenGL(PandiaView& v);

	// shows options for saving images and trajectory
	void window_SaveImageTrajectory(Model& m);

	// shows information about the firm
	void window_AboutUs();

	//shows deprecated infos.
	void window_bottomInfo();

	//depricated window
	void window_oldInfoWindow();

}