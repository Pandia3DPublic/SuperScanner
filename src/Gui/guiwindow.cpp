#include "guiwindow.h"
#include "utils/coreutil.h"
#include "core/integrate.h"
namespace PandiaGui {

	
	//spans menubar across whole GLFWWindow
	void window_Menubar(PandiaView& v) {
		ImGui::Begin("WholeWindow", NULL, PandiaGui::window_flags); //start menu bar

		ImGuiID dockspace_id = ImGui::GetID("DockSpace_Main");
		ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None | ImGuiDockNodeFlags_AutoHideTabBar);

		int oldLanguageChoice = PandiaGui::languageChoice;

		if (ImGui::BeginMainMenuBar()) {
			if (ImGui::BeginMenu(PandiaGui::lan_settings.c_str())) {
				if (ImGui::BeginMenu(PandiaGui::lan_language.c_str())) {
					ImGui::RadioButton(PandiaGui::lan_english.c_str(), &PandiaGui::languageChoice, 0);
					ImGui::RadioButton(PandiaGui::lan_german.c_str(), &PandiaGui::languageChoice, 1);
					ImGui::EndMenu();
				}
				ImGui::EndMenu();
			}

			ImGui::Separator();

			if (ImGui::BeginMenu("Shader")) {
				ImGui::RadioButton(PandiaGui::lan_noShader.c_str(), &PandiaGui::shadertyp, -1);
				ImGui::RadioButton(PandiaGui::lan_simpleShader.c_str(), &PandiaGui::shadertyp, 0);
				ImGui::RadioButton("Phong Shader", &PandiaGui::shadertyp, 1);
				ImGui::RadioButton(PandiaGui::lan_metalPhongShader.c_str(), &PandiaGui::shadertyp, 2);
				ImGui::Separator();
				ImGui::MenuItem(PandiaGui::lan_wireframe.c_str(), NULL, &v.wireframe);
				ImGui::EndMenu();
			}

			ImGui::Separator();

			if (ImGui::BeginMenu(PandiaGui::lan_help.c_str())) {
				ImGui::MenuItem(PandiaGui::lan_InfoModeText.c_str(), NULL, &PandiaGui::menu_InfoMode);
#ifdef DEVELOPERTOOLS
				ImGui::MenuItem("Show DemoWindow", NULL, &PandiaGui::menu_DemoWindow);
				ImGui::MenuItem("Show DebugWindow", NULL, &PandiaGui::menu_DebugWindow);
#endif //DEVELOPTERTOOLS
				ImGui::MenuItem(PandiaGui::lan_cameraPath.c_str(), NULL, &PandiaGui::menu_showCameraPath);
				ImGui::Separator();
				ImGui::MenuItem(PandiaGui::lan_aboutUs.c_str(), NULL, &PandiaGui::menu_AboutUs);
				ImGui::EndMenu();
			}
		}

		//changes language only one time after selecting in menu
		if (oldLanguageChoice != PandiaGui::languageChoice) {
			PandiaGui::setGuiLanguage(PandiaGui::languageChoice);
		}

		ImGui::End();
	}


	// shows warmUp indicator for starting reconrun
	void window_WarmUp() {
		if (threadCom::g_warmupCamera) {
			static float progress = 0.f;
			ImGui::Begin("warmupWindow", NULL, PandiaGui::subWindow_flags | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration);
			ImVec2 miniWindow(160.f, 50.f);
			ImGui::SetWindowSize(miniWindow);
			ImGui::SetWindowPos(ImVec2(topleft.x + renderWindowSize.x / 2 - miniWindow.x / 2, topleft.y + renderWindowSize.y / 2 - miniWindow.y));
			ImGui::TextColored(PandiaGui::col_yellow, PandiaGui::lan_warmUp.c_str(),0);
			progress = g_warmupProgress / 60.f;
			ImGui::ProgressBar(progress, ImVec2(-1.f, 0.f)); //shows progressbar how much frames are skipped
			ImGui::End();
		}
	}


	// shows if no camera is currently connected to device
	void window_CameraConnection() {
		if (!threadCom::g_cameraConnected && !threadCom::g_cameraParameterSet) {
			ImGui::SetNextWindowBgAlpha(0.7f);
			ImGui::Begin("cameraLostWindow", NULL, PandiaGui::subWindow_flags | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration);
			ImGui::SetWindowFontScale(1.5f);
			ImVec2 miniWindow(ImGui::CalcTextSize(PandiaGui::lan_noConnection.c_str()).x + 2 * ImGui::GetStyle().WindowPadding.x, -1.f);
			ImGui::SetWindowSize(miniWindow);
			ImGui::SetWindowPos(ImVec2(PandiaGui::topleft.x + PandiaGui::renderWindowSize.x / 2 - miniWindow.x / 2, PandiaGui::topleft.y + PandiaGui::renderWindowSize.y / 2 - miniWindow.y));
			ImGui::TextColored(PandiaGui::col_red_3, PandiaGui::lan_noConnection.c_str(),0);
			ImGui::End();
		}
	}


	// shows currently executing postprocessing-option and progess
	void window_PostProcessing() {
		if (threadCom::g_postProcessing) {

			ImGui::Begin("postprocessingWindow", NULL, PandiaGui::subWindow_flags | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration);
			ImVec2 miniWindow(0.f, 0.f);

			if (PandiaGui::pp_denseAlign) {

			}
			else if (PandiaGui::pp_voxelLength) {
				miniWindow = ImVec2(ImGui::CalcTextSize(PandiaGui::lan_voxelLength.c_str()).x + 2 * ImGui::GetStyle().WindowPadding.x, 90.f);
				ImGui::TextColored(PandiaGui::col_yellow, PandiaGui::lan_voxelLength.c_str(),0);
				float progress =PandiaGui::currentProgress / PandiaGui::totalPPWork;
				ImGui::ProgressBar(progress, ImVec2(-1.f, 0.f)); //shows user progressbar in Postprocessing
				ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_red_1);
				ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_red_2);
				ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_red_3);
				if (ImGui::Button(PandiaGui::lan_cancel.c_str(), ImVec2(-1, -1))) {
					PandiaGui::stopPostProcessing = true;
				}
				ImGui::PopStyleColor(3);
			}
			else if (PandiaGui::pp_meshReduction) {
				miniWindow = ImVec2(ImGui::CalcTextSize(PandiaGui::lan_meshReduction.c_str()).x + 2 * ImGui::GetStyle().WindowPadding.x, -1.f);
				ImGui::TextColored(PandiaGui::col_yellow, PandiaGui::lan_meshReduction.c_str(),0);
			}
			ImGui::SetWindowSize(miniWindow);
			ImGui::SetWindowPos(ImVec2(PandiaGui::topleft.x + PandiaGui::renderWindowSize.x / 2 - miniWindow.x / 2, PandiaGui::topleft.y + PandiaGui::renderWindowSize.y / 2 - miniWindow.y));
			ImGui::End();
		}
	}

	
	// shows camera legend of camera path visualisation
	void window_CameraLegend(Model& m) {
		if (PandiaGui::menu_showCameraPath && !m.chunks.empty() && threadCom::g_programState == threadCom::gui_PAUSE) {
			ImGuiStyle& style = ImGui::GetStyle();
			ImGuiStyle style_old = style;
			style.Colors[ImGuiCol_WindowBg] = ImVec4(0.3f, 0.3f, 0.3f, 1.f);

			int buffer = 10;
			ImGui::Begin("CameraLegende", NULL, PandiaGui::subWindow_flags | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoDecoration);
			ImDrawList* drawlist = ImGui::GetWindowDrawList();
			ImVec2 miniWindow(335.f, 50.f);
			ImGui::SetWindowPos(ImVec2(PandiaGui::topleft.x + buffer, PandiaGui::topleft.y + PandiaGui::renderWindowSize.y - miniWindow.y - buffer));
			ImGui::SetWindowSize(miniWindow);
			//ImGui::Text("camera legend");

			//third element of legend
			ImVec2 p = ImGui::GetCursorScreenPos();
			//p = ImVec2(p.x, p.y + 20);
			ImU32 blue = ImGui::GetColorU32(ImVec4(0.2f, 0.4f, 1.0f, 1.0f));
			ImU32 white = ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
			drawlist->AddRectFilledMultiColor(p, ImVec2(p.x + ImGui::GetWindowWidth() - 15, p.y + 15), white, blue, blue, white);
			p = ImVec2(p.x, p.y + 20);

			//centers text to window, stupid GUI needs too much operations for this
			ImVec2 textSize = ImGui::CalcTextSize(PandiaGui::lan_cameraLegend.c_str());
			textSize = ImVec2(textSize.x / 2, textSize.y);
			int width = ImGui::GetWindowWidth() / 2;
			//p = ImVec2(p.x + width - textSize.x, p.y);
			ImGui::SetCursorScreenPos(p);
			ImGui::TextUnformatted(PandiaGui::lan_cameraLegend.c_str());
			ImGui::End();
			style = style_old;
		}
	}


	// shows MainViewPort of rendered Mesh and tracking indicator
	void window_OpenGL(PandiaView& v) {
		ImGui::Begin("OpenGL Output and Indicator", NULL, ImGuiWindowFlags_AlwaysAutoResize);
		ImGui::BeginChild("Red");
		ImGui::ImageButton((void*)(intptr_t)v.texColorBufferID, ImVec2(PandiaGui::renderWindowSize.x, PandiaGui::renderWindowSize.y), ImVec2(0, 0), ImVec2(-1, 1), 0);
		if (!threadCom::g_current_slam_finished && threadCom::g_programState == threadCom::gui_RUNNING) { //only show indicator if reconstruction is in progress 
			ImDrawList* drawlist = ImGui::GetWindowDrawList();
			if (!threadCom::g_trackingLost) { //tracking ok
				drawlist->AddCircleFilled(ImVec2(PandiaGui::topleft.x + 40.f, PandiaGui::topleft.y + 40.f), 30.f, ImColor(PandiaGui::col_brightGreen), 30);
				//drawlist->AddCircle(ImVec2(topleft.x + 40.f, topleft.y + 40.f), 30.f, ImColor(PandiaGui::col_darkgreen), 30, 3.5f);
				//drawlist->AddLine(ImVec2(topleft.x + 25.f, topleft.y + 40.f), ImVec2(topleft.x + 40.f, topleft.y + 50.f), ImColor(PandiaGui::col_darkgreen), 6.f);
				//drawlist->AddLine(ImVec2(topleft.x + 38.f, topleft.y + 52.f), ImVec2(topleft.x + 55.f, topleft.y + 30.f), ImColor(PandiaGui::col_darkgreen), 6.f);
			}
			else { //tracking lost
				drawlist->AddCircleFilled(ImVec2(PandiaGui::topleft.x + 40.f, PandiaGui::topleft.y + 40.f), 30.f, ImColor(PandiaGui::col_red_1), 30);
				//drawlist->AddLine(ImVec2(topleft.x + 40.f, topleft.y + 20.f), ImVec2(topleft.x + 40.f, topleft.y + 45.f), ImColor(PandiaGui::col_orange), 8.f);
				//drawlist->AddCircleFilled(ImVec2(topleft.x + 40.f, topleft.y + 55.f), 4.5f, ImColor(PandiaGui::col_orange), 15);
			}
		}

		ImVec2 pos = ImGui::GetWindowSize();
		ImVec2 imageSize(PandiaGui::logo_widht,PandiaGui::logo_height);
		int offset = 15;
		ImGui::SetCursorPos(ImVec2(pos.x - imageSize.x - offset, pos.y - imageSize.y - offset));
		ImGui::Image((void*)(intptr_t)PandiaGui::logo_texture, imageSize);

		ImGui::EndChild();
		PandiaGui::renderWindowSize = ImGui::GetItemRectSize();
		PandiaGui::topleft = ImGui::GetItemRectMin();
		ImGui::End();
	}


	// shows save options for images and trajectory
	void window_SaveImageTrajectory(Model& m) {

#ifdef RECORDBUFFER
		ImGui::Checkbox(PandiaGui::lan_BoxSaveImages.c_str(), &PandiaGui::saveImageFlag);

		if (PandiaGui::saveImageFlag) {
			ImGui::Spacing();
			ImGui::TextUnformatted(PandiaGui::lan_InfoImageSave.c_str());
			ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_yellow);
			ImGui::TextWrapped(PandiaGui::saveImagePath.c_str(),0);
			ImGui::PopStyleColor();


			if (ImGui::Button(PandiaGui::lan_DirectoryImageSave.c_str())) {
				ImGui::OpenPopup("Save Image to:");
				PandiaGui::InfoMarker(PandiaGui::lan_DirectoryImageSaveInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
				PandiaGui::file_dialogSaveImage.current_path = PandiaGui::getTopDirectoryPath(PandiaGui::saveImagePath); //sets FileDialog to custom path
			}
			if (ImGui::Button(PandiaGui::lan_ButtonSaveImages.c_str())) {
				saveImagestoDisc(PandiaGui::saveImagePath, m.recordbuffer);
			}

		}
#endif

#ifdef SAVETRAJECTORY
		ImGui::Checkbox("Save Trajectory", &PandiaGui::saveTrajectoryFlag);

		if (PandiaGui::saveTrajectoryFlag) {
			ImGui::Spacing();
			ImGui::Text("Your Trajectory will be saved to location:");
			ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_yellow);
			ImGui::TextWrapped(PandiaGui::saveTrajectoriesPath.c_str(),0);
			ImGui::PopStyleColor();


			if (ImGui::Button("Choose directory saving directory")) {
				ImGui::OpenPopup("Save Trajectory to:");
				PandiaGui::InfoMarker("Click button, to browse and choose your location, where you want to save your data", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
				PandiaGui::file_dialogSaveImage.current_path = PandiaGui::getTopDirectoryPath(PandiaGui::saveTrajectoriesPath); //sets FileDialog to custom path

			}
			if (ImGui::Button("Save Trajectory###button")) {
				saveTrajectorytoDisk(PandiaGui::saveTrajectoriesPath, m);
			}

		}

		ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
#endif

	}


	void window_AboutUs() {

		if (PandiaGui::menu_AboutUs) {
			ImGui::OpenPopup(PandiaGui::lan_aboutUs.c_str());
			ImGui::BeginPopupModal(PandiaGui::lan_aboutUs.c_str(), NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);
			
			ImVec2 pos = ImGui::GetCursorPos();
			ImVec2 popupSize(350, -1);

			if (ImGui::InvisibleButton("##closePopupModal", popupSize)) {
				PandiaGui::menu_AboutUs = false;
				return;
			}

			int offset = 15;
			ImVec2 imagepos = ImVec2(pos.x + (popupSize.x - PandiaGui::logoWhite_width)/2, pos.y + offset);
			ImVec2 imageSize(PandiaGui::logoWhite_width, PandiaGui::logoWhite_height);

			ImGui::SetCursorPos(imagepos);
			ImGui::Image((void*)(intptr_t)PandiaGui::logoWhite_texture, imageSize);
			ImGui::Spacing();
			ImGui::Separator();
			ImGui::Spacing();
			ImGui::Text("Pandia3D");
			ImGui::Spacing();
			ImGui::Text("Inhaber: Tristan Kenneweg, M.Sc.");
			ImGui::Spacing();
			ImGui::Text(u8"� Pandia3D GmbH 2020");

			ImGui::End();
			ImGui::EndPopup();
			
		}
	}

	void window_bottomInfo(){
		if (PandiaGui::menu_InfoMode) {

		  ImGui::Begin("info window", NULL, PandiaGui::subWindow_flags);

		  ImGui::BeginTabBar("InfoTabBar", ImGuiTabBarFlags_None);
		  ImVec2 pos = ImGui::GetCursorPos();

		  if (ImGui::BeginTabItem("Session")) {

		    int integrationcounter = pandia_integration::integratedframes.size();
		    int framecounter = frame_id_counter.id;
		    float effectivity;

		    if (framecounter == 0)
		      effectivity = 0;
		    else
		      effectivity = integrationcounter / (float)framecounter;
		    // ImGui::Spacing();
		    // ImGui::Text("Session timer:            %s", clock.getClock().c_str());
		    ImGui::Spacing();
		    ImGui::Text("Frame counter:            %d", framecounter);
		    ImGui::Spacing();
		    ImGui::Text("Effective frames:         %d", integrationcounter);
		    ImGui::Spacing();
		    ImGui::Text("Effectivity of recording:");
		    ImGui::SameLine();

		    if (effectivity >= 0.9) {
		      ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_brightGreen);
		    }
		    else if (0.8 < effectivity && effectivity < 0.9) {
		      ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_yellow);
		    }
		    else {
		      ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_red_2);
		    }
		    ImGui::Text("%.1f %%", 100 * effectivity);
		    ImGui::PopStyleColor();

		    ImGui::EndTabItem();
		  }

		  if (ImGui::BeginTabItem("History")) {
		    ImGui::EndTabItem();
		  }

		  // Pandia-Logo
		  float logo_width = 200.f;
		  float logo_height = logo_width / PandiaGui::logo_aspect;

		  int offset = 15;
		  float windowheight = ImGui::GetWindowHeight() - pos.y - offset;
		  float windowwidth = ImGui::GetWindowWidth();

		  if ((logo_height) > windowheight) {
		    logo_height = windowheight;
		    logo_width = PandiaGui::logo_aspect * logo_height;
		    //ImGui::SetCursorPosX(windowwidth - widthtmp - 15.f);
		  }

		  ImGui::SetCursorPos(ImVec2(ImGui::GetWindowWidth() - logo_width - offset, pos.y + 5));
		  //ImGui::Image((void*)(intptr_t)PandiaGui::logo_texture, ImVec2((int)logo_width, (int)logo_height));

		  ImGui::EndTabBar();

		  ImGui::End();
		}
	}



	// depricated. Only for referencing purposes
	void window_oldInfoWindow() {

		ImGui::Begin("info window", NULL, PandiaGui::subWindow_flags);

		ImGui::Columns(2, "bottomcolumns2", false);
		float columnseparation = 200;
		ImGui::SetColumnWidth(ImGui::GetColumnIndex(), ImGui::GetWindowWidth() - columnseparation - 25);
		if (PandiaGui::menu_InfoMode) {

			//ImGui::Text("%s", clock.getClock());

			if (g_camType == camtyp::typ_data) {
				ImGui::TextColored(PandiaGui::col_yellow, "INFO: You are currently in LOADING-MODE");
				ImGui::TextColored(PandiaGui::col_yellow, "INFO: You use picture data from your hard drive.");
			}
			else {
				ImGui::TextColored(PandiaGui::col_yellow, "INFO: You are currently in LIVE-MODE");
				ImGui::TextColored(PandiaGui::col_yellow, "INFO: You WON'T SAVE Images");
			}

		}

		ImGui::NextColumn();

		// Pandia-Logo
		float widthtmp = columnseparation;
		float heighttmp = widthtmp / PandiaGui::logo_aspect;

		float windowheight = ImGui::GetWindowHeight();
		float windowwidth = ImGui::GetWindowWidth();

		if ((heighttmp) > windowheight) {
			heighttmp = windowheight;
			widthtmp = PandiaGui::logo_aspect * heighttmp;
			ImGui::SetCursorPosX(windowwidth - widthtmp - 15.f);
		}
		ImGui::SetCursorPosY((windowheight - heighttmp) / 2.f);
		ImGui::Image((void*)(intptr_t)PandiaGui::logo_texture, ImVec2((int)widthtmp, (int)heighttmp), ImVec2(0,0), ImVec2(1,1), ImVec4(1,1,1,1), ImVec4(1,1,1,1));
		ImGui::End();

	}


}//namespace PandiaGui












