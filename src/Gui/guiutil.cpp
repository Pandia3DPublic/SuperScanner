#include "guiutil.h"
#include "utils/coreutil.h"
#include "core/integrate.h"
#include "core/threadCom.h"

using namespace open3d;
using namespace std;

namespace PandiaGui
{
	vector<float> reconTimes;
	double reconTimeLast = 0.0;
	double reconTimeMax = 0.0;
	double reconTimeAvg = 0.0;

	int shadertyp = 0;
	//menu, saving and loading flags
	bool menu_InfoMode = true;	  //user can hover over info icons
	bool menu_DemoWindow = false; //shows imgui demo windows
	bool menu_DebugWindow = true;
	bool menu_ShaderDefault = true;
	bool menu_ShaderWireFrame = false;
	bool menu_ShaderNeutral = false;
	bool menu_AboutUs = false;

	bool loadDataFlag = false; //if true reads data from harddrive
	bool saveImageFlag = false;
	bool saveTrajectoryFlag = false;
	int loadOptionSwitch = 1; //status int. Status 0 is load mesh, status 1 is load images
	bool disableInput = false;
	bool nonemptyBuffer = false;

	bool menu_showCameraPath = false;
	bool cameraPathBuild = false;
	//window flags
	const ImGuiWindowFlags window_flags = /*ImGuiWindowFlags_MenuBar |*/ ImGuiWindowFlags_NoDocking |
										  ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoResize |
										  ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar;
	const ImGuiWindowFlags subWindow_flags = ImGuiWindowFlags_NoCollapse;
	//internal flag for gui-popups
	const ImGuiWindowFlags disableInput_flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
												ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav;

	//colors
	ImVec4 const col_darkgreen((ImVec4)ImColor::HSV(0.36f, 0.75f, 0.45f));
	ImVec4 const col_green((ImVec4)ImColor::HSV(0.286f, 0.55f, 0.55f));
	ImVec4 const col_lime((ImVec4)ImColor::HSV(0.286f, 0.75f, 0.75f));
	ImVec4 const col_brightGreen((ImVec4)ImColor::HSV(0.3f, 0.9f, 0.75f));
	ImVec4 const col_yellow((ImVec4)ImColor::HSV(0.15f, 0.75f, 0.75f));
	ImVec4 const col_orange((ImVec4)ImColor::HSV(0.1f, 0.75f, 0.85f));
	ImVec4 const col_gray((ImVec4)ImColor::HSV(0.f, 0.f, 0.35f));
	ImVec4 const col_green_1((ImVec4)ImColor::HSV(0.286f, 0.5f, 0.5f));
	ImVec4 const col_green_2((ImVec4)ImColor::HSV(0.286f, 0.6f, 0.6f));
	ImVec4 const col_green_3((ImVec4)ImColor::HSV(0.286f, 0.7f, 0.7f));
	ImVec4 const col_red_1((ImVec4)ImColor::HSV(0.f, 0.55f, 0.55f));
	ImVec4 const col_red_2((ImVec4)ImColor::HSV(0.f, 0.7f, 0.7f));
	ImVec4 const col_red_3((ImVec4)ImColor::HSV(0.f, 0.85f, 0.85f));

	//filedialog variables (pathes for loading and saving mesh/images)
	string saveMeshPath = "C:/dev/";
	string selectedMeshPath = "C:/dev/";
	string saveImagePath = "C:/dev/"; //save images here
	string saveTrajectoriesPath = "C:/dev/";

	imgui_addons::ImGuiFileBrowser file_dialogLoadMesh;
	imgui_addons::ImGuiFileBrowser file_dialogLoadImage;
	imgui_addons::ImGuiFileBrowser file_dialogSaveImage;
	imgui_addons::ImGuiFileBrowser file_dialogSaveMesh;
	bool fileDialogOpenNow = false;

	//image variables
	GLuint infoSymbol_texture = 0;
	GLuint logo_texture = 0;
	GLuint logoWhite_texture = 0;
	GLuint background_image = 0;
	int logo_widht = 0;
	int logo_height = 0;
	int logoWhite_width = 0;
	int logoWhite_height = 0;
	float logo_aspect = 0;

	//Text language variables
	int languageChoice = 0;
	//menu
	string lan_settings;
	string lan_language;
	string lan_english;
	string lan_german;
	string lan_noShader;
	string lan_simpleShader;
	string lan_metalPhongShader;
	string lan_wireframe;
	string lan_help;
	string lan_InfoModeText;
	string lan_cameraPath;
	string lan_raycasting;
	string lan_aboutUs;

	//control window
	string lan_loadData;
	string lan_loadDataInfo;
	string lan_loadMesh;
	string lan_ButtonLoadMesh;
	string lan_loadMeshInfo;
	string lan_loadImages;
	string lan_loadImagesInfo;
	string lan_loadMeshDirectory;
	string lan_loadMeshDirectoryInfo;
	string lan_loadMeshTextHeader;
	string lan_loadMeshTextHeaderInfo;
	string lan_loadImagesDirectory;
	string lan_loadImagesDirectoryInfo;
	string lan_loadImagesTextHeader;
	string lan_loadImagesTextHeaderInfo;
	//GUI-States
	string lan_InfoAzure;
	string lan_InfoRealsense;
	string lan_InfoDataCam;
	string lan_ButtonStart;
	string lan_AnimationText;
	string lan_ButtonStartDummy;
	string lan_ButtonPause;
	string lan_ButtonResume;
	string lan_ButtonSaveMesh;
	string lan_ButtonClearMesh;
	string lan_BoxSaveImages;
	string lan_InfoImageSave;
	string lan_DirectoryImageSave;
	string lan_DirectoryImageSaveInfo;
	string lan_ButtonSaveImages;
	string lan_TextPostProcessing;
	string lan_textPostProcessingInfo;
	string lan_meshReductionInfo;
	string lan_voxelLenghtInfo;
	string lan_ButtonStartPostProcessing;
	//GUI-Popups
	string lan_warmUp;
	string lan_voxelLength;
	string lan_cancel;
	string lan_meshReduction;
	string lan_noConnection;
	string lan_cameraLegend;

	//Window variables
	ImVec2 topleft;
	ImVec2 renderWindowSize;
	ImVec2 buttonsize;

	//postprocessing variables
	bool stopPostProcessing = false; //cancel flag by button press
	float totalPPWork = 0.f;
	float currentProgress = 0.f;
	bool pp_denseAlign = false;
	bool pp_meshReduction = false;
	bool pp_filterTaubin = false;
	bool pp_nonLinearOpt = false;
	bool pp_denseo3dOpt = false;
	bool pp_voxelLength = false;
	bool postpro_disable = true;
	float voxelSlider = 1.0f;
	float meshSlider = 100;
	float minSlider = 0.01f;
	float maxSlider = 0.05f;

	//miscellaneous variables
	char animationBuffer[25];
	shared_ptr<geometry::TriangleMesh> loadedMesh = make_shared<geometry::TriangleMesh>();
	ImVec2 absolutWindowPos;

	//#################################### functions ################################

	void initPandiaGui()
	{

		//reads last save/load locations of user
		ifstream lastpath_imageLoad("lastpath_imageLoad.txt");
		if (lastpath_imageLoad.is_open())
		{
			getline(lastpath_imageLoad, g_readimagePath);
		}
		lastpath_imageLoad.close();

		ifstream lastpath_imageSave("lastpath_imageSave.txt");
		if (lastpath_imageSave.is_open())
		{
			getline(lastpath_imageSave, saveImagePath);
		}
		lastpath_imageSave.close();

		ifstream lastpath_meshLoad("lastpath_meshLoad.txt");
		if (lastpath_meshLoad.is_open())
		{
			getline(lastpath_meshLoad, PandiaGui::selectedMeshPath);
		}
		lastpath_meshLoad.close();

		ifstream lastpath_meshSave("lastpath_meshSave.txt");
		if (lastpath_meshSave.is_open())
		{
			getline(lastpath_meshSave, PandiaGui::saveMeshPath);
		}
		lastpath_meshSave.close();

		//load info symbol textures from file
		int infoSymbol_width = 0;
		int infoSymbol_height = 0;
		bool infoSymbol = PandiaGui::LoadTextureFromFile("textures/infosymbol.png", &infoSymbol_texture, &infoSymbol_width, &infoSymbol_height);
		IM_ASSERT(infoSymbol);

		//load logo texture form file
		bool logo = PandiaGui::LoadTextureFromFile("textures/PandiaLogo.png", &logo_texture, &logo_widht, &logo_height);
		IM_ASSERT(logo);
		logo_aspect = logo_widht / (float)logo_height;

		//load logo texture with white font from file
		bool logo_white = PandiaGui::LoadTextureFromFile("textures/Logo_Pandia3_white_Font.png", &logoWhite_texture, &logoWhite_width, &logoWhite_height);
		IM_ASSERT(logo);

		//load background image from file
		int background_width = 0;
		int background_height = 0;
		bool background = PandiaGui::LoadTextureFromFile("textures/background.png", &background_image, &background_width, &background_height);
		IM_ASSERT(background);


		readPandiaGuiIni();

		PandiaGui::setGuiLanguage(PandiaGui::languageChoice);

		renderWindowSize = ImVec2(g_initial_width, g_initial_height);
	}

	void InfoMarker(const char *desc, bool &enabled, GLuint texture)
	{
		if (enabled)
		{
			ImGui::SameLine();
			ImGui::Image((void *)(intptr_t)texture, ImVec2(16, 16));
			if (ImGui::IsItemHovered())
			{
				ImGui::BeginTooltip();
				ImGui::TextUnformatted(desc);
				ImGui::EndTooltip();
			}
		}
	}

	void GuiBlockItem(bool disable)
	{
		if (disable)
		{

			auto pos = ImGui::GetCursorScreenPos();
			auto height = ImGui::GetWindowSize();
			ImGui::SetNextWindowBgAlpha(0.5f);
			ImGui::SetNextWindowSize(ImVec2(ImGui::GetWindowWidth() - 2 * ImGui::GetStyle().WindowPadding.x, -1)); //30 is height of max Widget to cover full
			ImVec2 childSize = ImVec2(ImGui::GetWindowWidth() - 2 * ImGui::GetStyle().WindowPadding.x, 32.f);

			//length of pop-up spans over whole sub-window
			pos.x -= ImGui::GetStyle().FramePadding.x;
			pos.y -= ImGui::GetStyle().FramePadding.y;
			ImGui::SetNextWindowPos(pos);
			ImGuiContext *context = ImGui::GetCurrentContext();

			//ImGui needs unique name for new window creation
			std::string s = std::to_string(pos.y);
			ImGui::Begin(s.c_str(), &disable, disableInput_flags);
			ImGui::SetCurrentContext(context);
			ImGui::SetWindowPos(pos);
			if (ImGui::IsWindowHovered())
			{
				ImGui::SetTooltip("Please clear mesh first");
			}

			ImGui::End();
		}
	}

	void GuiBlockItem(bool disable, ImVec2 size)
	{
		if (disable)
		{
			auto pos = ImGui::GetCursorScreenPos();
			pos.y = pos.y - size.y / 2;
			pos.x -= ImGui::GetStyle().FramePadding.x; //align BlockElement with Edges of window
			auto padding = ImGui::GetStyle().FramePadding;

			//Originposition of Button in ImGui is in vertical middle of button itself, padding and rounding error-padding included
			pos.y = pos.y - ((size.y) / 2 + 3 * padding.y);
			ImGui::SetNextWindowBgAlpha(0.5f);

			//length of pop-up spans over whole sub-window
			ImGui::SetNextWindowSize(ImVec2(ImGui::GetWindowWidth() - 2 * ImGui::GetStyle().WindowPadding.x, size.y + 2 * padding.y));
			ImGui::SetNextWindowPos(pos);

			//ImGui needs unique name for new window creation
			std::string s = std::to_string(pos.y);
			ImGui::Begin(s.c_str(), &disable, disableInput_flags);
			ImGui::End();
		}
	}


	//Only works if directory is navigates via "/"
	std::string getTopDirectoryPath(std::string &path)
	{
		std::string tmp = path;
		while (tmp.back() == '/')
		{
			tmp.pop_back();
		}
		while (tmp.back() == '\\')
		{
			tmp.pop_back();
		}
		string tmp2 = tmp;
		tmp.resize(tmp.find_last_of("/") + 1);
		if (tmp.size() == 0)
		{
			tmp = tmp2;
			tmp.resize(tmp.find_last_of("\\") + 1);
		}

		//to enable this current_path must be changed from private to public
		return tmp; //sets FileDialog to custom path
	}

	bool LoadTextureFromFile(const char *filename, GLuint *out_texture, int *out_width, int *out_height)
	{
		// Create a OpenGL texture identifier
		glGenTextures(1, out_texture);

		// Load from file
		unsigned char *image_data = stbi_load(filename, out_width, out_height, NULL, 0);
		if (image_data == NULL)
			return false;

		glBindTexture(GL_TEXTURE_2D, *out_texture);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, *out_width, *out_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

		// Setup filtering parameters for display
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(image_data);

		return true;
	}

	// 0 = english
	// 1 = german
	void setGuiLanguage(int choice)
	{

		ifstream en_File("languageFolder//en.txt");
		ifstream de_File("languageFolder//de.txt");
		ifstream *streamptr = NULL;

		list<string> nameList;
		list<string> translation;

		switch (choice)
		{
		case 0:
			streamptr = &en_File;
			break;

		case 1:
			streamptr = &de_File;
			break;
		default:
			streamptr = &en_File;
			break;
		}

		if (streamptr->is_open())
		{

			string line;
			while (getline(*streamptr, line))
			{

				if (line[0] == '#' || line.empty())
					continue;
				auto itr = line.begin();
				advance(itr, line.find_first_of("\""));
				line.erase(std::remove_if(line.begin(), itr, ::isspace), itr);

				auto delimiterPos = line.find("=");
				auto name = line.substr(0, delimiterPos);
				auto size = line.find_last_of("\"") - line.find_first_of("\"") - 1;
				auto value = line.substr(line.find_first_of("\"") + 1, size);
				//auto value = line.substr(delimiterPos + 2);
				//value.pop_back();
				assignTranslation(name, value);
				//cout << name << "-" << value << "|" << endl;
			}
		}
		else
		{
			cout << "no language text-file found" << endl;
		}
	}

	void assignTranslation(string &name, string &translation)
	{

		if (name == "lan_settings")
		{
			lan_settings = translation;
			return;
		}
		if (name == "lan_language")
		{
			lan_language = translation;
			return;
		}
		if (name == "lan_english")
		{
			lan_english = translation;
			return;
		}
		if (name == "lan_german")
		{
			lan_german = translation;
			return;
		}
		if (name == "lan_noShader")
		{
			lan_noShader = translation;
			return;
		}
		if (name == "lan_simpleShader")
		{
			lan_simpleShader = translation;
			return;
		}
		if (name == "lan_metalPhongShader")
		{
			lan_metalPhongShader = translation;
			return;
		}
		if (name == "lan_wireframe")
		{
			lan_wireframe = translation;
			return;
		}
		if (name == "lan_help")
		{
			lan_help = translation;
			return;
		}
		if (name == "lan_InfoModeText")
		{
			lan_InfoModeText = translation;
			return;
		}
		if (name == "lan_cameraPath")
		{
			lan_cameraPath = translation;
			return;
		}
		if (name == "lan_raycasting")
		{
			lan_raycasting = translation;
			return;
		}
		if (name == "lan_aboutUs")
		{
			lan_aboutUs = translation;
			return;
		}

		//control window
		if (name == "lan_loadData")
		{
			lan_loadData = translation;
			return;
		}
		if (name == "lan_loadDataInfo")
		{
			lan_loadDataInfo = translation;
			return;
		}
		if (name == "lan_loadMesh")
		{
			lan_loadMesh = translation;
			return;
		}
		if (name == "lan_ButtonLoadMesh")
		{
			lan_ButtonLoadMesh = translation;
			return;
		}
		if (name == "lan_loadMeshInfo")
		{
			lan_loadMeshInfo = translation;
			return;
		}
		if (name == "lan_loadImages")
		{
			lan_loadImages = translation;
			return;
		}
		if (name == "lan_loadImagesInfo")
		{
			lan_loadImagesInfo = translation;
			return;
		}
		if (name == "lan_loadMeshDirectory")
		{
			lan_loadMeshDirectory = translation;
			return;
		}
		if (name == "lan_loadMeshDirectoryInfo")
		{
			lan_loadMeshDirectoryInfo = translation;
			return;
		}
		if (name == "lan_loadMeshTextHeader")
		{
			lan_loadMeshTextHeader = translation;
			return;
		}
		if (name == "lan_loadMeshTextHeaderInfo")
		{
			lan_loadMeshTextHeaderInfo = translation;
			return;
		}
		if (name == "lan_loadImagesDirectory")
		{
			lan_loadImagesDirectory = translation;
			return;
		}
		if (name == "lan_loadImagesDirectoryInfo")
		{
			lan_loadImagesDirectoryInfo = translation;
			return;
		}
		if (name == "lan_loadImagesTextHeader")
		{
			lan_loadImagesTextHeader = translation;
			return;
		}
		if (name == "lan_loadImagesTextHeaderInfo")
		{
			lan_loadImagesTextHeaderInfo = translation;
			return;
		}

		//GUI-States
		if (name == "lan_InfoAzure")
		{
			lan_InfoAzure = translation;
			return;
		}
		if (name == "lan_InfoRealsense")
		{
			lan_InfoRealsense = translation;
			return;
		}
		if (name == "lan_InfoDataCam")
		{
			lan_InfoDataCam = translation;
			return;
		}
		if (name == "lan_InfoDataCam")
		{
			lan_InfoDataCam = translation;
			return;
		}
		if (name == "lan_ButtonStart")
		{
			lan_ButtonStart = translation;
			return;
		}
		if (name == "lan_AnimationText")
		{
			lan_AnimationText = translation;
			return;
		}
		if (name == "lan_ButtonStartDummy")
		{
			lan_ButtonStartDummy = translation;
			return;
		}
		if (name == "lan_ButtonPause")
		{
			lan_ButtonPause = translation;
			return;
		}
		if (name == "lan_ButtonResume")
		{
			lan_ButtonResume = translation;
			return;
		}
		if (name == "lan_ButtonSaveMesh")
		{
			lan_ButtonSaveMesh = translation;
			return;
		}
		if (name == "lan_ButtonClearMesh")
		{
			lan_ButtonClearMesh = translation;
			return;
		}
		if (name == "lan_BoxSaveImages")
		{
			lan_BoxSaveImages = translation;
			return;
		}
		if (name == "lan_InfoImageSave")
		{
			lan_InfoImageSave = translation;
			return;
		}
		if (name == "lan_DirectoryImageSave")
		{
			lan_DirectoryImageSave = translation;
			return;
		}
		if (name == "lan_DirectoryImageSaveInfo")
		{
			lan_DirectoryImageSaveInfo = translation;
			return;
		}
		if (name == "lan_ButtonSaveImages")
		{
			lan_ButtonSaveImages = translation;
			return;
		}
		if (name == "lan_TextPostProcessing")
		{
			lan_TextPostProcessing = translation;
			return;
		}
		if (name == "lan_textPostProcessingInfo")
		{
			lan_textPostProcessingInfo = translation;
			return;
		}
		if (name == "lan_voxelLength")
		{
			lan_voxelLength = translation;
			return;
		}
		if (name == "lan_voxelLenghtInfo")
		{
			lan_voxelLenghtInfo = translation;
			return;
		}
		if (name == "lan_meshReduction")
		{
			lan_meshReduction = translation;
			return;
		}
		if (name == "lan_meshReductionInfo")
		{
			lan_meshReductionInfo = translation;
			return;
		}
		if (name == "lan_ButtonStartPostProcessing")
		{
			lan_ButtonStartPostProcessing = translation;
			return;
		}

		//GUI-Popups
		if (name == "lan_warmUp")
		{
			lan_warmUp = translation;
			return;
		}
		if (name == "lan_cancel")
		{
			lan_cancel = translation;
			return;
		}
		if (name == "lan_meshReduction")
		{
			lan_meshReduction = translation;
			return;
		}
		if (name == "lan_noConnection")
		{
			lan_noConnection = translation;
			return;
		}
		if (name == "lan_cameraLegend")
		{
			lan_cameraLegend = translation;
			return;
		}

		//lan_voxelLength = "voxel length";
	}

	// saves values of PandiaGui variables upon closing program
	void createPandiaGuiIni()
	{
		ofstream iniFile("pandiagui.ini");
		iniFile << "### This saves the current user-settings of the GUI" << endl;
		iniFile << "language = " << languageChoice << endl;
		iniFile << "shader = " << shadertyp << endl;
		iniFile << "info-modus = " << menu_InfoMode << endl;
		iniFile << "camerapath = " << menu_showCameraPath << endl;
		iniFile << "loadData = " << loadDataFlag << endl;
		iniFile << "loadOption = " << loadOptionSwitch << endl;

#ifdef DEVELOPERTOOLS
		iniFile << endl;
		iniFile << "### This saves the current developer-settings of the GUI" << endl;
		iniFile << "imgui_demo-window = " << menu_DemoWindow << endl;
		iniFile << "pandiagui_debug-window = " << menu_DebugWindow << endl;
#endif // DEVELOPERTOOLS

		iniFile.close();
	}

	//assignes values from readPandiaGuiIni
	void assignIniValues(string &s, int i)
	{

		if (s == "language")
		{
			languageChoice = i;
		}
		if (s == "shader")
		{
			shadertyp = i;
		}
		if (s == "info-modus")
		{
			menu_InfoMode = i;
		}
		if (s == "camerapath")
		{
			menu_showCameraPath = i;
		}
		if (s == "loadData")
		{
			loadDataFlag = i;
		}
		if (s == "loadOption")
		{
			loadOptionSwitch = i;
		}

#ifdef DEVELOPERTOOLS
		if (s == "imgui_demo-window")
		{
			menu_DemoWindow = i;
		}
		if (s == "pandiagui_debug-window")
		{
			menu_DebugWindow = i;
		}
#endif // DEVELOPERTOOLS
	}

	//sets PandiaGui variables to last saved values in INI
	void readPandiaGuiIni()
	{
		ifstream iniFile("pandiagui.ini");

		if (iniFile.is_open())
		{

			string line;
			while (getline(iniFile, line))
			{
				line.erase(std::remove_if(line.begin(), line.end(), ::isspace),
						   line.end());
				if (line[0] == '#' || line.empty())
					continue;
				auto delimiterPos = line.find("=");
				auto name = line.substr(0, delimiterPos);
				auto value = line.substr(delimiterPos + 1);
				int i = std::stoi(value);
				assignIniValues(name, i);
			}
			cout << "PandiaIni successfully read" << endl;
		}
		else
		{
			cout << "No PandiaIni found" << endl;
		}
	}

	// checks if any fileDialog is active and shows correlated window
	template <typename SpaceType>
	void handle_FileDialogs(SpaceType& space, Model &m)
	{

		if (file_dialogLoadMesh.showFileDialog("Load Mesh", imgui_addons::ImGuiFileBrowser::DialogMode::OPEN, ImVec2(700.f, 310.f), ".ply,.obj,.glb,.gltf"))
		{
			selectedMeshPath = file_dialogLoadMesh.selected_path;
			ofstream lastpath_mesh("lastpath_meshLoad.txt");
			lastpath_mesh << selectedMeshPath << endl;
			lastpath_mesh.close();
		}

		if (file_dialogLoadImage.showFileDialog("Load Images From", imgui_addons::ImGuiFileBrowser::DialogMode::SELECT, ImVec2(700.f, 310.f)))
		{
			g_readimagePath = file_dialogLoadImage.selected_path;
			ofstream lastpath_image("lastpath_imageLoad.txt");
			lastpath_image << g_readimagePath << endl;
			lastpath_image.close();
			if (fileExists(g_readimagePath + "intrinsic.txt"))
			{
				utility::LogDebug("Set intrinsics from file\n");
				setFromIntrinsicFile(g_readimagePath + "intrinsic.txt");
			}
			else
			{
				utility::LogWarning("No intrinsic file found. Taking default intrinsic\n");
				setDefaultIntrinsic();
			}
			m.globalGrid->setIntrinsic(space,g_intrinsic);
		}

#ifdef RECORDBUFFER
		if (file_dialogSaveImage.showFileDialog("Save Image to:", imgui_addons::ImGuiFileBrowser::DialogMode::SELECT, ImVec2(700.f, 310.f)))
		{
			saveImagePath = file_dialogSaveImage.selected_path;
			ofstream lastpath_image("lastpath_imageSave.txt");
			lastpath_image << saveImagePath << endl;
			lastpath_image.close();
		}
#endif
#ifdef SAVETRAJECTORY

		if (file_dialogSaveImage.showFileDialog("Save Trajectory to:", imgui_addons::ImGuiFileBrowser::DialogMode::SELECT, ImVec2(700.f, 310.f)))
		{
			saveTrajectoriesPath = file_dialogSaveImage.selected_path;
		}
#endif
		//this is the mesh saving dialog
		if (file_dialogSaveMesh.showFileDialog("Save mesh to", imgui_addons::ImGuiFileBrowser::DialogMode::SAVE, ImVec2(700.f, 310.f), ".ply,.obj,.glb,.gltf"))
		{
			saveMeshPath = file_dialogSaveMesh.selected_path;
			ofstream lastpath_mesh("lastpath_meshSave.txt");
			lastpath_mesh << saveMeshPath << endl;
			lastpath_mesh.close();
			m.saveCPUMesh(saveMeshPath + file_dialogSaveMesh.ext);
		}
	}

	template void handle_FileDialogs<Kokkos::Cuda>(Kokkos::Cuda &space, Model &m);

#ifdef DEVELOPERTOOLS
		void showDebugWindow(Model &m, PandiaView &v, double GUItime)
	{

		// static PandiaClock t(true);
		bool open = true;
		ImGui::Begin("Debug", &open, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_AlwaysAutoResize);

		//Model related information
		if (ImGui::CollapsingHeader("MODEL"))
		{
			int nchunks = m.chunks.size();
			ImGui::TextColored(PandiaGui::col_orange, "MODEL");
			ImGui::Text("Chunks (m.chunks.size()): %i", (int)nchunks);
			ImGui::Text("Invalid Chunks (m.invalidChunks.size()): %i", (int)m.invalidChunks.size());

			ImGui::Text("Filtered Matches (m.filteredmatches.x.size()): %i",(int) m.pairTransforms.x.size());
			// ImGui::Text("Raw Matches (m.rawmatches.x.size()): %i", (int)m.rawmatches.x.size());
			ImGui::Text("Frame-ID (frame_id_counter.id): %i", frame_id_counter.id);
			ImGui::Text("Chunk-ID (chunk_id_counter.id): %i", chunk_id_counter.id);
			int tmpgpuframes = Frame::nGPUFrames;
			ImGui::Text("Frames in GPU Memory %i", tmpgpuframes);

			if (ImGui::TreeNode("TSDF volume"))
			{

				auto tmpSubvolumes = m.globalGrid->voxelMap2.size();
				ImGui::Text("Active subvolumes (m.globalGrid->voxelMap2.size()): %d", tmpSubvolumes);
				ImGui::Text("fx: %.2f", m.globalGrid->fx);
				ImGui::Text("fy: %.2f", m.globalGrid->fy);
				ImGui::Text("cx:  %.2f", m.globalGrid->cx);
				ImGui::Text("cy:  %.2f", m.globalGrid->cy);
				ImGui::Separator();
				ImGui::TreePop();
				ImGui::Separator();
			}

			if (ImGui::TreeNode("Visualisation"))
			{

				ImVec2 histogramsize(300, 150);
				int no_frames = 0;
				int max_orbChunk = 0;
				int max_orbFrame = 0;
				int max_efficientMatches = 0;
				float avg_orbkeypointsChunk = 0;
				float avg_orbkeypointsFrame = 0;
				float avg_efficientMatchChunk = 0;
				vector<float> vec_orbkeypointsChunk;
				vector<float> vec_orbkeypointsFrame;
				vector<float> vec_efficientMatchChunk;
				vector<int> vec_frameID;

				//saves orbkeypoint-size of chunks for graphic
				for (int i = 0; i < nchunks; i++)
				{
					vec_orbkeypointsChunk.push_back(m.chunks[i]->orbKeypoints.size());
					avg_orbkeypointsChunk += vec_orbkeypointsChunk.back();

					if (max_orbChunk < vec_orbkeypointsChunk[i])
						max_orbChunk = vec_orbkeypointsChunk[i];

					no_frames = m.chunks[i]->frames.size();

					//saves orbkeypoint-size of frames for graphic
					for (int j = 0; j < no_frames; j++)
					{
						vec_orbkeypointsFrame.push_back(m.chunks[i]->frames[j]->orbKeypoints.size());
						avg_orbkeypointsFrame += vec_orbkeypointsFrame.back();

						if (max_orbFrame < vec_orbkeypointsFrame.back())
							max_orbFrame = vec_orbkeypointsFrame.back();
					}
				}

				avg_orbkeypointsChunk /= nchunks;
				avg_orbkeypointsFrame /= vec_orbkeypointsFrame.size();

				//show graphics
				ImGui::PlotHistogram("###Keypoints Frame", vec_orbkeypointsFrame.data(), vec_orbkeypointsFrame.size(), 0, "orbKeypoints (Frame)", 0.0f, max_orbFrame, histogramsize);
				ImGui::SameLine();
				ImGui::PlotHistogram("###Keypoints Chunk", vec_orbkeypointsChunk.data(), vec_orbkeypointsChunk.size(), 0, "orbKeypoints (Chunk)", 0.0f, max_orbChunk, histogramsize);
				ImGui::SameLine();
				ImGui::PlotHistogram("###efficient matches of chunk", vec_efficientMatchChunk.data(), vec_efficientMatchChunk.size(), 0, "efficient Matches of Chunks", 0.0f, 400, histogramsize);

				ImGui::BeginChild("### for better layout of columns", ImVec2(0, 80), true, ImGuiWindowFlags_AlwaysAutoResize);
				ImGui::Text("number of ...");
				ImGui::Columns(3, "separation", false);
				ImGui::Text("cur Frames: %d", (int)vec_orbkeypointsFrame.size());
				ImGui::Text("max orbkeypoints: %d", max_orbFrame);
				ImGui::Text("avg orbkeypoints: %.2f", avg_orbkeypointsFrame);

				ImGui::NextColumn();
				ImGui::Text("cur Chunks: %d", (int)vec_orbkeypointsChunk.size());
				ImGui::Text("max orbkeypoints: %d", max_orbChunk);
				ImGui::Text("avg orbkeypoints: %.2f", avg_orbkeypointsChunk);

				ImGui::NextColumn();
				ImGui::Text("cur Chunks: %d", (int)vec_efficientMatchChunk.size());
				ImGui::Text("max efficient matches: %d", max_efficientMatches);
				ImGui::Text("avg efficient matches: %.2f", avg_efficientMatchChunk);
				ImGui::EndChild();
				ImGui::TreePop();
				ImGui::Separator();
			}

			if (ImGui::TreeNode("Misc"))
			{
				ImGui::TreePop();
				ImGui::Separator();
			}
		}

		if (ImGui::CollapsingHeader("TIMINGS"))
		{
			if (threadCom::g_programState == threadCom::gui_RUNNING && g_reconChunkItTimer != reconTimeLast) {
				reconTimeLast = g_reconChunkItTimer;
				reconTimes.push_back(reconTimeLast);
				// max
				if (reconTimeLast > reconTimeMax) {
					reconTimeMax = reconTimeLast;
				}
				// average
				if (reconTimes.size() > 1) 
					reconTimeAvg = (reconTimeAvg + reconTimeLast) / 2.0;
				else
					reconTimeAvg = reconTimeLast;
			}

			//show graphics
			ImVec2 histogramsize(300, 150);
			ImGui::PlotHistogram("###Reconrun chunk times", reconTimes.data(), reconTimes.size(), 0, "time in s (reconChunk)", 0.0f, reconTimeMax, histogramsize);
			ImGui::Text("g_reconChunkItTimer: %f", g_reconChunkItTimer);
			ImGui::Text("last time: %f", reconTimeLast);
			ImGui::Text("max time: %f", reconTimeMax);
			ImGui::Text("avg time: %f", reconTimeAvg);

			// reset after clear mesh
			if (threadCom::g_programState == threadCom::gui_READY) {
				reconTimes = vector<float>();
				reconTimeLast = 0.0;
				reconTimeMax = 0.0;
				reconTimeAvg = 0.0;
				g_reconChunkItTimer = 0.0;
			}
		}

		if (ImGui::CollapsingHeader("PANDIA_INTEGRATION"))
		{

			ImGui::Text("nonemptyBuffer: %d", PandiaGui::nonemptyBuffer);
			ImGui::Text("integrationBuffer.size(): %d", (int)pandia_integration::integrationBuffer.size());
			ImGui::Text("reintegrationBuffer.size(): %d", (int)pandia_integration::reintegrationBuffer.size());
			ImGui::Text("deintegrationBuffer.size(): %d", (int)pandia_integration::deintegrationBuffer.size());
			ImGui::Text("integratedframes.size(): %d", (int)pandia_integration::integratedframes.size());
			ImGui::Text("threadCom::unmeshed_data: %d", (bool)threadCom::unmeshed_data);
		}

		if (ImGui::CollapsingHeader("THREADS VARIABLESS"))
		{

			ImGui::Text("g_current_slam_finished: %d", (bool)threadCom::g_current_slam_finished);
			ImGui::Text("g_trackingLost: %d", (bool)threadCom::g_trackingLost);
			ImGui::Text("g_reconThreadFinished: %d", (bool)threadCom::g_reconThreadFinished);
			ImGui::Text("g_postProcessing: %d", (bool)threadCom::g_postProcessing);
		}

		if (ImGui::CollapsingHeader("CAMERA"))
		{

			ImGui::Text("current Camera Typ:");
			ImGui::SameLine();
			switch (g_camType)
			{
			case camtyp::typ_client:
				ImGui::Text("Client");
				break;
			case camtyp::typ_data:
				ImGui::Text("Data");
				break;
			case camtyp::typ_kinect:
				ImGui::Text("Kinect");
				break;
			case camtyp::typ_realsense:
				ImGui::Text("Realsense");
				break;
			default:
				break;
			}

			if (ImGui::TreeNode("current Camera Position (m.currentPos)"))
			{
				Eigen::Matrix4d camPos = m.currentPos; //prone to Error, without currentposLock
				ImGui::Text("%*f, %*f, %*f, %*f", 10, camPos(0, 0), 10, camPos(0, 1), 10, camPos(0, 2), 10, camPos(0, 3));
				ImGui::Text("%*f, %*f, %*f, %*f", 10, camPos(1, 0), 10, camPos(1, 1), 10, camPos(1, 2), 10, camPos(1, 3));
				ImGui::Text("%*f, %*f, %*f, %*f", 10, camPos(2, 0), 10, camPos(2, 1), 10, camPos(2, 2), 10, camPos(2, 3));
				ImGui::Text("%*f, %*f, %*f, %*f", 10, camPos(3, 0), 10, camPos(3, 1), 10, camPos(3, 2), 10, camPos(3, 3));
				ImGui::TreePop();
				ImGui::Separator();
			}

			if (ImGui::TreeNode("g_intrinsic"))
			{
				ImGui::Text("fx: %.2f", g_intrinsic.intrinsic_matrix_(0, 0));
				ImGui::Text("fy: %.2f", g_intrinsic.intrinsic_matrix_(1, 1));
				ImGui::Text("w:  %.2f", g_intrinsic.intrinsic_matrix_(0, 2));
				ImGui::Text("h:  %.2f", g_intrinsic.intrinsic_matrix_(1, 2));
				ImGui::Text("camera width:  %d", g_intrinsic.width_);
				ImGui::Text("camera height: %d", g_intrinsic.height_);
				ImGui::TreePop();
				ImGui::Separator();
			}
		}

		if (ImGui::CollapsingHeader("GUI"))
		{

			string state = "";

			switch (threadCom::g_programState)
			{
			case 0:
				state = "READY";
				break;
			case 1:
				state = "RUNNING";
				break;
			case 2:
				state = "PAUSE";
				break;
			default:
				break;
			}

			ImGui::Text("loadDataFlag: % d", PandiaGui::loadDataFlag);
			ImGui::Text("loadOptionSwitch: %d", PandiaGui::loadOptionSwitch);
			ImGui::Text("current Program State: %s", state.c_str());

			if (ImGui::TreeNode("File_Dialog"))
			{

				ImGui::Text("g_readImagePath: %s", g_readimagePath.c_str());

				if (ImGui::TreeNode("Load Image"))
				{
					ImGui::Text("currentpath: %s", PandiaGui::file_dialogLoadImage.current_path.c_str());
					ImGui::Text("selectedpath: %s", PandiaGui::file_dialogLoadImage.selected_path.c_str());
					ImGui::Text("file-name: %s", PandiaGui::file_dialogLoadImage.selected_fn.c_str());
					ImGui::Text("file-extension: %s", PandiaGui::file_dialogLoadImage.ext.c_str());
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("Load Mesh"))
				{
					ImGui::Text("currentpath: %s", PandiaGui::file_dialogLoadMesh.current_path.c_str());
					ImGui::Text("selectedpath: %s", PandiaGui::file_dialogLoadMesh.selected_path.c_str());
					ImGui::Text("file-name: %s", PandiaGui::file_dialogLoadImage.selected_fn.c_str());
					ImGui::Text("file-extension: %s", PandiaGui::file_dialogLoadImage.ext.c_str());
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("Save Image"))
				{
					ImGui::Text("currentpath: %s", PandiaGui::file_dialogSaveImage.current_path.c_str());
					ImGui::Text("selectedpath: %s", PandiaGui::file_dialogSaveImage.selected_path.c_str());
					ImGui::Text("file-name: %s", PandiaGui::file_dialogSaveImage.selected_fn.c_str());
					ImGui::Text("file-extension: %s", PandiaGui::file_dialogSaveImage.ext.c_str());
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("Save Mesh"))
				{
					ImGui::Text("currentpath: %s", PandiaGui::file_dialogSaveMesh.current_path.c_str());
					ImGui::Text("selectedpath: %s", PandiaGui::file_dialogSaveMesh.selected_path.c_str());
					ImGui::Text("file-name: %s", PandiaGui::file_dialogSaveMesh.selected_fn.c_str());
					ImGui::Text("file-extension: %s", PandiaGui::file_dialogSaveMesh.ext.c_str());
					ImGui::TreePop();
					ImGui::Separator();
				}

				ImGui::TreePop();
			}
		}

		if (ImGui::CollapsingHeader("PANDIAVIEW"))
		{

			if (ImGui::TreeNode("Matrices"))
			{

				if (ImGui::TreeNode("Model"))
				{
					Eigen::Matrix4f model = v.model_matrix_;
					ImGui::Text("%*f, %*f, %*f, %*f", 10, model(0, 0), 10, model(0, 1), 10, model(0, 2), 10, model(0, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, model(1, 0), 10, model(1, 1), 10, model(1, 2), 10, model(1, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, model(2, 0), 10, model(2, 1), 10, model(2, 2), 10, model(2, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, model(3, 0), 10, model(3, 1), 10, model(3, 2), 10, model(3, 3));
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("View"))
				{
					Eigen::Matrix4f view = v.view_matrix_;
					ImGui::Text("%*f, %*f, %*f, %*f", 10, view(0, 0), 10, view(0, 1), 10, view(0, 2), 10, view(0, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, view(1, 0), 10, view(1, 1), 10, view(1, 2), 10, view(1, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, view(2, 0), 10, view(2, 1), 10, view(2, 2), 10, view(2, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, view(3, 0), 10, view(3, 1), 10, view(3, 2), 10, view(3, 3));
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("Projection"))
				{
					Eigen::Matrix4f projection = v.projection_matrix_;
					ImGui::Text("%*f, %*f, %*f, %*f", 10, projection(0, 0), 10, projection(0, 1), 10, projection(0, 2), 10, projection(0, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, projection(1, 0), 10, projection(1, 1), 10, projection(1, 2), 10, projection(1, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, projection(2, 0), 10, projection(2, 1), 10, projection(2, 2), 10, projection(2, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, projection(3, 0), 10, projection(3, 1), 10, projection(3, 2), 10, projection(3, 3));
					ImGui::TreePop();
					ImGui::Separator();
				}

				if (ImGui::TreeNode("MVP"))
				{
					Eigen::Matrix4f mvp = v.MVP_matrix_;
					ImGui::Text("%*f, %*f, %*f, %*f", 10, mvp(0, 0), 10, mvp(0, 1), 10, mvp(0, 2), 10, mvp(0, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, mvp(1, 0), 10, mvp(1, 1), 10, mvp(1, 2), 10, mvp(1, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, mvp(2, 0), 10, mvp(2, 1), 10, mvp(2, 2), 10, mvp(2, 3));
					ImGui::Text("%*f, %*f, %*f, %*f", 10, mvp(3, 0), 10, mvp(3, 1), 10, mvp(3, 2), 10, mvp(3, 3));
					ImGui::TreePop();
				}

				ImGui::TreePop();
				ImGui::Separator();
			}

			if (ImGui::TreeNode("Window"))
			{
				ImGui::Text("window width: %d", v.window_width_);
				ImGui::Text("window heigth: %d", v.window_height_);
				ImGui::Text("view ratio: %f", v.view_ratio_);
				ImGui::Text("Aspect: %f", v.aspect_);
				ImGui::TreePop();
				ImGui::Separator();
			}

			if (ImGui::TreeNode("Camera"))
			{
				ImGui::Text("FOV: %f", v.field_of_view_);
				ImGui::Text("virtual FOV: %f", v.virtual_fov);
				ImGui::Text("distance: %f", v.distance_);
				ImGui::Text("zoom: %f", v.zoom_);
				ImGui::Text("z_near: %f", v.z_near_);
				ImGui::Text("z_far: %f", v.z_far_);
				ImGui::Text("const z_near: %f", v.constant_z_near_);
				ImGui::Text("const z_far: %f", v.constant_z_far_);
				Eigen::Vector3d camPos = v.eye_;
				ImGui::Text("virtual Cameraposition: %*f, %*f, %*f", 10, camPos(0), 10, camPos(1), 10, camPos(2));

				ImGui::TreePop();
				ImGui::Separator();
			}
		}
		if (ImGui::CollapsingHeader("Scaling"))
		{
			ImGui::Text("Active Subvolumes: %i", m.globalGrid->voxelMap2.size());
			ImGui::Text("Max Subvolumes: %i", m.globalGrid->maxVoxelBlocks);
			ImGui::Text("Number of frames in g_framebuffer: %i", (int)threadCom::g_framebuffer.size());
			ImGui::Text("Number of frames in g_rawbuffer: %i", (int)threadCom::g_rawbuffer.size());
			ImGui::Text("Number of elements in pairTransforms in model: %i", (int)m.pairTransforms.x.size());
			ImGui::Text("Number of frames in GPU Memory: %i", (int)Frame::nGPUFrames);
			ImGui::Text("Time per gui loop run: %f", GUItime);
			ImGui::Text("Time per recon loop run: %f", g_reconIterationTimer);
			ImGui::Text("Time per recon Chunk loop: %f", g_reconChunkItTimer);
		}

		ImGui::End();
		}

#endif //DEVELOPERTOOLS

	shared_ptr<geometry::TriangleMesh> createCameraScaffolding(shared_ptr<GLGeometry> &cameraPath, int &divider)
	{

		auto &mesh = cameraPath->mesh_;

		for (int i = 0; i < divider; i++)
		{
			mesh->vertex_colors_[i] = Eigen::Vector3d(0.0, 0.0, 0.9);
		}

		for (int i = divider; i < mesh->vertices_.size(); i++)
		{
			mesh->vertex_colors_[i] = Eigen::Vector3d(0.9, 0.1, 0.1);
		}

		return mesh;
	}

} //namespace PandiaGui