#pragma once

#include "Gui/PandiaView.h"
#include "core/Model.h"
#include "tinygltf/stb_image.h"
#include "../postprocessing/postprocessing.h"
#include "core/threadCom.h"
#include "cameras/CameraThreadandInit.h"


//contains util functions and variables!
//todo maybe make this singleton class
namespace PandiaGui {

	extern int shadertyp;			 //-1 is non, 0 is simple, 1 pandia, 2 is o3d,
	extern int meshShaderType;			 //-1 is non, 0 is simple, 1 pandia, 2 is o3d
	extern bool menu_showCameraPath; //only indicates menu
	extern bool cameraPathBuild; //indicates if mesh needs to be build

	//menu, saving and loading flags
	extern bool menu_InfoMode; //user can hover over info icons
	extern bool menu_DemoWindow; //shows imgui demo window
	extern bool menu_DebugWindow;
	extern bool menu_showCameraPath;
	extern bool menu_ShaderDefault;
	extern bool menu_ShaderWireFrame;
	extern bool menu_ShaderNeutral;
	extern bool menu_AboutUs;
	
	extern bool loadDataFlag; //if true reads data from harddrive
	extern bool saveImageFlag;
	extern bool saveTrajectoryFlag;
	extern int loadOptionSwitch; //status int. Status 0 is load mesh, status 1 is load images
	extern bool disableInput;
	extern bool nonemptyBuffer;

	//window flags
	extern const ImGuiWindowFlags window_flags; //flags for whole application window
	extern const ImGuiWindowFlags subWindow_flags; //flags for control-/ and info window


	//colors 
	extern const ImVec4 col_darkgreen;
	extern const ImVec4 col_green;
	extern const ImVec4 col_lime;
	extern const ImVec4 col_brightGreen;
	extern const ImVec4 col_yellow;
	extern const ImVec4 col_orange;
	extern const ImVec4 col_gray;
	extern const ImVec4 col_green_1;
	extern const ImVec4 col_green_2;
	extern const ImVec4 col_green_3;
	extern const ImVec4 col_red_1;
	extern const ImVec4 col_red_2;
	extern const ImVec4 col_red_3;


	//filedialog variables (pathes for loading and saving mesh/images)
	extern std::string saveMeshPath;
	extern std::string selectedMeshPath;
	extern std::string saveImagePath;
	extern std::string saveTrajectoriesPath;
	extern imgui_addons::ImGuiFileBrowser file_dialogLoadMesh;
	extern imgui_addons::ImGuiFileBrowser file_dialogLoadImage;
	extern imgui_addons::ImGuiFileBrowser file_dialogSaveImage;
	extern imgui_addons::ImGuiFileBrowser file_dialogSaveMesh;
	extern bool fileDialogOpenNow;


	//image variables
	extern GLuint infoSymbol_texture;
	extern GLuint logo_texture;
	extern GLuint logoWhite_texture;
	extern GLuint background_image;
	extern int logo_widht;
	extern int logo_height;
	extern int logoWhite_width;
	extern int logoWhite_height;
	extern float logo_aspect;


	//Text language variables
	extern int languageChoice;
	//menu
	extern std::string lan_settings;
	extern std::string lan_language;
	extern std::string lan_english;
	extern std::string lan_german;
	extern std::string lan_noShader;
	extern std::string lan_simpleShader;
	extern std::string lan_metalPhongShader;
	extern std::string lan_wireframe;
	extern std::string lan_help;
	extern std::string lan_InfoModeText;
	extern std::string lan_cameraPath;
	extern std::string lan_raycasting;
	extern std::string lan_aboutUs;
	//control window
	extern std::string lan_loadData;
	extern std::string lan_loadDataInfo;
	extern std::string lan_loadMesh;
	extern std::string lan_ButtonLoadMesh;
	extern std::string lan_loadMeshInfo;
	extern std::string lan_loadImages;
	extern std::string lan_loadImagesInfo;
	extern std::string lan_loadMeshDirectory;
	extern std::string lan_loadMeshDirectoryInfo;
	extern std::string lan_loadMeshTextHeader;
	extern std::string lan_loadMeshTextHeaderInfo;
	extern std::string lan_loadImagesDirectory;
	extern std::string lan_loadImagesDirectoryInfo;
	extern std::string lan_loadImagesTextHeader;
	extern std::string lan_loadImagesTextHeaderInfo;
	//GUI-States
	extern std::string lan_InfoAzure;
	extern std::string lan_InfoRealsense;
	extern std::string lan_InfoDataCam;
	extern std::string lan_ButtonStart;
	extern std::string lan_AnimationText;
	extern std::string lan_ButtonStartDummy;
	extern std::string lan_ButtonPause;
	extern std::string lan_ButtonResume;
	extern std::string lan_ButtonSaveMesh;
	extern std::string lan_ButtonClearMesh;
	extern std::string lan_BoxSaveImages;
	extern std::string lan_InfoImageSave;
	extern std::string lan_DirectoryImageSave;
	extern std::string lan_DirectoryImageSaveInfo;
	extern std::string lan_ButtonSaveImages;
	extern std::string lan_TextPostProcessing;
	extern std::string lan_textPostProcessingInfo;
	extern std::string lan_voxelLength;
	extern std::string lan_voxelLenghtInfo;
	extern std::string lan_meshReduction;
	extern std::string lan_meshReductionInfo;
	extern std::string lan_ButtonStartPostProcessing;
	//GUI-Popups
	extern std::string lan_warmUp;
	extern std::string lan_voxelLength;
	extern std::string lan_cancel;
	extern std::string lan_meshReduction;
	extern std::string lan_noConnection;
	extern std::string lan_cameraLegend;


	//window variables
	extern ImVec2 topleft;
	extern ImVec2 renderWindowSize;
	extern ImVec2 buttonsize;


	//postprocessing variables
	extern bool stopPostProcessing; //cancel flag by button press
	extern float totalPPWork;
	extern float currentProgress;
	extern bool pp_denseAlign;
	extern bool pp_meshReduction;
	extern bool pp_voxelLength;
	extern bool pp_filterTaubin;
	extern bool pp_nonLinearOpt;
	extern bool pp_denseo3dOpt;
	extern bool postpro_disable;
	extern 	float voxelSlider;
	extern float meshSlider;
	extern 	float minSlider;
	extern 	float maxSlider;


	//miscellaneous variables
	extern char animationBuffer[25]; //small animation for "searching camera"
	extern std::shared_ptr<open3d::geometry::TriangleMesh> loadedMesh; //loaded mesh will be renderd in this variable
	extern ImVec2 absolutWindowPos; // variable for GuiBlockItem()
	extern bool showRaycast;



//#################################### functions ################################


	void initPandiaGui();
	

	void InfoMarker(const char* desc, bool& enabled, GLuint texture);
	
	//must be called before text-sized entity
	void GuiBlockItem(bool disable);

	//must be called before any ImGui-Widget
	//Spawns semi-transparent Window to prevent User-interaction
	//must be called after the acutal button, or whatever!!!!!!!
	//NOTE: keyboard shortcuts will circumvent this
	void GuiBlockItem(bool disable, ImVec2 size);

	std::string getTopDirectoryPath(std::string& path);

	bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int* out_width, int* out_height);


	void setGuiLanguage(int choice);
	void assignTranslation(std::string &name, std::string &translation);

	//INI-functions
	void createPandiaGuiIni();
	void readPandiaGuiIni();
	void assignIniValues(std::string &s, int i);

	//renders WindowsElements
	

	//
	template <typename SpaceType>
	void handle_FileDialogs(SpaceType &space,Model &m);

#ifdef DEVELOPERTOOLS
	void showDebugWindow(Model &m, PandiaView &v, double GUItime);
#endif // DEVELOPERTOOLS

	std::shared_ptr<open3d::geometry::TriangleMesh> createCameraScaffolding(std::shared_ptr<GLGeometry> &cameraPath, int &divider);

} //namespace PandiaGui