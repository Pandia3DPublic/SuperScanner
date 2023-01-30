#include "core/reconrun.h"
#include "utils/coreutil.h"
#include "utils/visutil.h"
#include "Gui/Shader.h"
#include "cmakedefines.h"
#include "Gui/geometry.h"
#include "Gui/PandiaView.h"
#include "Gui/guiutil.h"
#include "Gui/guiwindow.h"
#include "postprocessing/postprocessing.h"
#include "readconfig.h"
#include "core/integrate.h"

using namespace std;
using namespace open3d;

void glfw_error_callback(int error, const char *description)
{
  fprintf(stderr, "Error: %s\n", description);
}

void InitWindow(GLFWwindow *&window, int &w_width, int &w_height)
{

  if (!glfwInit())
  {
    std::cout << "fatal error. glfw init not successfull \n";
  }

  glfwSetErrorCallback(glfw_error_callback);

  int visible = 1;
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_VISIBLE, visible);

  string window_name_ = "Pandia 3D Scanning Solution";

  window = glfwCreateWindow(w_width, w_height, window_name_.c_str(), NULL, NULL);
  if (!window)
  {
    utility::LogError("Failed to create window\n");
  }
  int left = 100;
  int top = 100;
  glfwSetWindowPos(window, left, top);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  //glfwSetFramebufferSizeCallback(window,framebuffer_size_callback);
}

static bool spacedown = false;

void processInput(GLFWwindow *window, Model &m)
{
  if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
  {
    glfwSetWindowShouldClose(window, true);
    std::cout << "close callback received \n";
  }
  bool set = false;
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE && spacedown)
  {
    if (threadCom::g_programState == threadCom::gui_RUNNING && !threadCom::g_reconThreadFinished)
    {
      std::cout << "stopping \n";
      threadCom::g_pause = true;
      threadCom::g_programState = threadCom::gui_PAUSE;
      set = true;
    }
    if (threadCom::g_programState == threadCom::gui_PAUSE && !threadCom::g_current_slam_finished && !set)
    {
      std::cout << "resuming \n";
      threadCom::g_pause = false;
      threadCom::g_programState = threadCom::gui_RUNNING; //programm runnign again
    }
  }

  spacedown = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
}

void initImGui(GLFWwindow *window)
{
  const char *glsl_version = "#version 130";
  IMGUI_CHECKVERSION();
  // Setup Platform/Renderer bindings
  ImGui::CreateContext();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);
  ImGui::StyleColorsDark();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  //Style
  ImGuiStyle &style = ImGui::GetStyle();
  style.WindowBorderSize = 0.0f;
  style.FrameRounding = 5.0f;
  style.FrameBorderSize = 0.f; //Prevents GuiBlock-Element from being smaller resized than actual window
}

int main()
{
  readconfig("config.txt");
  Kokkos::initialize();
  {

    Kokkos::Cuda superscannerSpace; //maybe create one instance here, if problems arise. Deauflt streams behaves differently
    //this might sync with everything

    utility::SetVerbosityLevel((open3d::utility::VerbosityLevel)g_verbosity);

    //init window
    GLFWwindow *window;
    InitWindow(window, g_initial_width, g_initial_height);

    //init opengl with glew here
    if (glewInit() != GLEW_OK)
    {
      std::cout << "something is very wrong! glew init failed \n";
      return false;
    }
    //init ImGUI  //order is critical here otherwise imgui is missing events
    PandiaView view(window);
    initImGui(window);
    PandiaGui::initPandiaGui(); //set variables, load textures, etc.

    //########################## shader stuff #######################
    string shaderpath = TOSTRING(SHADER_PATH);
    //get Shader from text files
    string vs_name = "o3d_phong_vertex_shader.txt";
    string fs_name = "o3d_phong_fragment_shader.txt";
    string gs_name = "";
    string vsp = shaderpath + vs_name;
    string fsp = shaderpath + fs_name;
    string gsp = shaderpath + gs_name;
    Shader o3dphongShader(vsp.c_str(), fsp.c_str());
    //o3dphongShader.use();

    Eigen::Matrix4f lightpos = Eigen::Matrix4f::Identity();
    o3dphongShader.setUniform4Matrix("light_position_world_4", lightpos); // set to zero position
    o3dphongShader.setUniform4Matrix("M", Eigen::Matrix4f::Identity());
    float unitlength = 0.70710678118 * 0.32;
    Eigen::Matrix4f neutralmatrix;
    neutralmatrix << unitlength, unitlength, unitlength, unitlength,
        unitlength, unitlength, unitlength, unitlength,
        unitlength, unitlength, unitlength, unitlength,
        1, 1, 1, 1;
    Eigen::Vector4f unitvec(unitlength, unitlength, unitlength, 1);
    o3dphongShader.setUniform4Matrix("light_color_4", neutralmatrix);
    o3dphongShader.setUniform4("light_diffuse_power_4", unitvec);
    o3dphongShader.setUniform4("light_specular_power_4", unitvec);
    o3dphongShader.setUniform4("light_specular_shininess_4", unitvec);
    o3dphongShader.setUniform4("light_ambient", unitvec);

    vs_name = "simple_vertex_shader.txt";
    fs_name = "simple_fragment_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    Shader simpleShader(vsp.c_str(), fsp.c_str());

    vs_name = "wireframe_vertex_shader.txt";
    fs_name = "wireframe_fragment_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    Shader wireFrameShader(vsp.c_str(), fsp.c_str());

    vs_name = "phong_vertex_shader.txt";
    fs_name = "phong_fragment_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    Shader phongShader(vsp.c_str(), fsp.c_str());
    phongShader.setVec3("lightColor", Eigen::Vector3f::Ones());
    Eigen::Vector3f ldir(0.0, -1.0, 0.3);
    ldir.normalize();
    phongShader.setVec3("lightDir", ldir);

    vs_name = "empty_vertex_shader.txt";
    fs_name = "empty_fragment_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    Shader emptyShader(vsp.c_str(), fsp.c_str());
    Eigen::Vector4f background_color(0.45f, 0.55f, 0.60f, 1.00f);
    emptyShader.setUniform4("backgroundColor", background_color);

    vs_name = "raycast_vertex_shader.txt";
    fs_name = "raycast_fragment_shader.txt";
    gs_name = "raycast_geometry_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    gsp = shaderpath + gs_name;
    // Shader raycastShader(vsp.c_str(), fsp.c_str(), gsp.c_str());
    // raycastShader.setVec3("lightColor", Eigen::Vector3f::Ones());
    // raycastShader.setVec3("lightPos", Eigen::Vector3f::Zero());
    // raycastShader.setVec3("lightColor", Eigen::Vector3f::Ones());
    // Eigen::Vector3f ldir(0.0, -1.0, 0.3);
    // ldir.normalize();
    // raycastShader.setVec3("lightDir", ldir);
    // raycastShader.setVec3("lightPos", Eigen::Vector3f(0.0f,10.0f,10.0f));

    vs_name = "experimental_vertex_shader.txt";
    fs_name = "experimental_fragment_shader.txt";
    vsp = shaderpath + vs_name;
    fsp = shaderpath + fs_name;
    Shader experimentalShader(vsp.c_str(), fsp.c_str());
    Eigen::Vector4f expColor(1.0f, 1.0f, 1.0f, 1.0f);

    //get opengl locations for shader inputs.
    //In all shaders the following location and naming convention must be held true:
    //layout (location = 0) in vec3 vertex_position;
    //layout (location = 1) in vec3 vertex_color;
    //layout (location = 2) in vec3 vertex_normal;
    GLuint vertex_location = 0;
    GLuint color_location = 1;
    GLuint normal_location = 2;

    //########################## end shader stuff #######################

    //build and init view object that handles camera position, opengl viewport, mvp matrix generation and framebuffer
    glViewport(0, 0, g_initial_width, g_initial_height);
    view.ChangeWindowSize(g_initial_width, g_initial_height);
    view.Reset();
    view.SetConstantZFar(2.5 * g_cutoff);
    view.SetConstantZNear(g_mincutoff);
    view.lookat_ = Eigen::Vector3d(0, 0, 0);
    view.front_ = Eigen::Vector3d(0, 0, -1);
    view.SetProjectionParameters();
    view.SetViewMatrices();
    view.genFramebuffer(); //generates a framebuffer for us to draw upon

    //set background color
    glClearColor(background_color(0), background_color(1), background_color(2), background_color(3));

    glEnable(GL_DEPTH_TEST);

    //####################################define imGUI util variables####################
    ImVec2 oldWindowSize(0, 0);
    bool resetAll = false;
    //####################################end define imGUI util variables####################

    Model m;
    threadCom::g_m = &m;
    thread reconstructionThread;
    shared_ptr<GLGeometry> cameraMesh;          //just cameras for each chunk position
    shared_ptr<GLGeometry> cameraConnectorMesh; // connector lines for drawing them red

    //todo think host mesh pipeline through. we shouls not do marching cubes here already!
    GLGeometry hostMesh(m.globalGrid->mesh, vertex_location, color_location, normal_location);
    // GLGeometry gpuRaycastImage(m.globalGrid->rayPcd, m.globalGrid->rayNormals, vertex_location, color_location, normal_location);
    GLGeometry gpuRaycastImage(m.globalGrid, vertex_location, color_location, normal_location);

    Shader *currentShaderPointer = &simpleShader;
    GLGeometry *currentGeometryPointer = &hostMesh; //either hostmest, or raycast stuff
    currentShaderPointer->use();

    Eigen::Matrix4d currentPosCopy = Eigen::Matrix4d::Identity(); //to copy the current position in thread locks
    GLuint o3dMVP = o3dphongShader.getUniformLocation("MVP");
    GLuint o3dV = o3dphongShader.getUniformLocation("V");
    GLuint simpleMVP = simpleShader.getUniformLocation("MVP");
    GLuint wireFrameMVP = wireFrameShader.getUniformLocation("MVP");
    GLuint phongMVP = phongShader.getUniformLocation("MVP");
    GLuint phongViewpos = phongShader.getUniformLocation("viewPos");
    // GLuint rayCastMVP = raycastShader.getUniformLocation("MVP");
    GLuint experimentalMVP = experimentalShader.getUniformLocation("MVP");

    Kokkos::Timer cleanMemoryTimer;

    //start threads
    thread cameraConnectionThread;
    cameraConnectionThread = thread(cameraConnectionThreadFunction); //check every second if camera is connected  and sets g_parameterset

    thread postProcessingThread;

    //################################### MAIN LOOP START #######################################
    //################################### MAIN LOOP START #######################################
    //################################### MAIN LOOP START #######################################
    //################################### MAIN LOOP START #######################################
    //################################### MAIN LOOP START #######################################

    Kokkos::Timer GUITimer;
    double GUIIteartionTime = 0;
    double lasttime = GUITimer.seconds();
    while (!glfwWindowShouldClose(window))
    {
      GUIIteartionTime = GUITimer.seconds() - lasttime; //get time of last iteration
      lasttime = GUITimer.seconds();
      if (resetAll)
      {
        //for deallocation
        pandia_integration::integratedframes.clear(); //doesnt happen otherwise
        pandia_integration::deintegrationBuffer.clear();
        pandia_integration::reintegrationBuffer.clear();
        pandia_integration::integrationBuffer.clear();
        //placement new for a clean slate
        m.~Model();
        new (&m) Model;
        m.globalGrid->setIntrinsic(superscannerSpace, g_intrinsic);
        hostMesh.~GLGeometry();
        new (&hostMesh) GLGeometry(m.globalGrid->mesh, vertex_location, color_location, normal_location);
        gpuRaycastImage.~GLGeometry();
        new (&gpuRaycastImage) GLGeometry(m.globalGrid, vertex_location, color_location, normal_location);

        resetAll = false;
      }
      //Only processes Input if no ImGui-FileDialog is open. Prevents closing, resuming, etc.
      //Prevents from actions while typing file names
      if (!PandiaGui::fileDialogOpenNow)
        processInput(window, m); //only close and toggle resume

      //check if pause was pressed and mesh needs to be generated
      //todo umneshed data should live in gpuvoxelgrid
      if (threadCom::g_programState == threadCom::gui_PAUSE && threadCom::unmeshed_data)
      {
        //check if integrationlists are empty
        pandia_integration::integrationlock.lock();
        bool noIntegrations = pandia_integration::integrationBuffer.empty() &&
                              pandia_integration::reintegrationBuffer.empty() &&
                              pandia_integration::deintegrationBuffer.empty();
        pandia_integration::integrationlock.unlock();
        if (noIntegrations)
        {
          std::this_thread::sleep_for(100ms); //todo stupid. Note that this needs some com with reconrun
          m.globalGrid->cleanMemory(superscannerSpace);
          cleanMemoryTimer.reset();
          cout << "Calling Marching Cubes \n";
          pandia_integration::tsdfLock.lock();
          m.globalGrid->marchingCubesHost();
          pandia_integration::tsdfLock.unlock();
          cout << "Marching cubes done \n";
          if (hostMesh.mesh_ != m.globalGrid->mesh){ //rebuilt in case globalgrid is newly build in pp
            cout <<"rebuilding host mesh\n";
            hostMesh.~GLGeometry();
            new (&hostMesh) GLGeometry(m.globalGrid->mesh, vertex_location, color_location, normal_location);
          }
          hostMesh.updateGeometry();
          currentGeometryPointer = &hostMesh;
        }
      }

      //check if we need to perform a raycast
      if (threadCom::g_programState == threadCom::gui_RUNNING)
      {                                     // do not set cam pos in post processing
        threadCom::g_currentposlock.lock(); //todo should be different lock (minor)
        currentPosCopy = m.currentPos;
        threadCom::g_currentposlock.unlock();
        view.setRealCPos(currentPosCopy);
        pandia_integration::tsdfLock.lock();
        m.globalGrid->raycast(superscannerSpace, currentPosCopy.cast<float>());
        pandia_integration::tsdfLock.unlock();

        currentShaderPointer = &simpleShader;
        currentGeometryPointer = &gpuRaycastImage;
        simpleShader.setUniform4Matrix(simpleMVP, view.GetMVPMatrix());
        int npixel = view.window_height_ * view.window_width_;
        int nres = g_resi * g_resj;
        glPointSize(npixel / nres + 1);

        //todo this only needs to happens when geometry changes, and only renew opengl buffers, also we dont always need to raycast
        currentGeometryPointer->updateGeometryRay();

        //clean memory every 10 seconds
        if (cleanMemoryTimer.seconds() > 10)
        {
          m.globalGrid->cleanMemory(superscannerSpace);
          cleanMemoryTimer.reset();
        }
      }
      else if (!threadCom::unmeshed_data)
      {
        //integer makes sense here for imgui compabtibility
        switch (PandiaGui::shadertyp)
        {
        case -1:
          currentShaderPointer = &emptyShader;
          break;
        case 0:
          currentShaderPointer = &simpleShader;
          simpleShader.setUniform4Matrix(simpleMVP, view.GetMVPMatrix());

          break;
        case 1:
          currentShaderPointer = &phongShader;
          phongShader.setUniform4Matrix(phongMVP, view.GetMVPMatrix());
          phongShader.setUniform4Matrix(phongViewpos, currentPosCopy.cast<float>());
          break;
        case 2:
          currentShaderPointer = &o3dphongShader;
          o3dphongShader.setUniform4Matrix(o3dMVP, view.GetMVPMatrix()); //give mvp matrix to opengl
          o3dphongShader.setUniform4Matrix(o3dV, view.GetViewMatrix());  //o3d
          break;
        default:
          break;
        }
      }


      glBindFramebuffer(GL_FRAMEBUFFER, view.framebufferID); //bind our framebuffer to render in it
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    //clears color- and depth buffer of OpenGL

      //todo check for cpu mesh load case
      //show camera path (only in valid states)
      if (PandiaGui::menu_showCameraPath && !m.chunks.empty() && threadCom::g_programState == threadCom::gui_PAUSE)
      {
        //build camera path once
        if (!PandiaGui::cameraPathBuild)
        {
          cameraMesh = make_shared<GLGeometry>(getCameraPathMesh(m), vertex_location, color_location);
          cameraConnectorMesh = make_shared<GLGeometry>(createPathMesh(m), vertex_location, color_location);
          PandiaGui::cameraPathBuild = true;
          std::cout << "building camera \n";
        }

        wireFrameShader.setUniform4Matrix(wireFrameShader.getUniformLocation("MVP"), view.GetMVPMatrix());
        simpleShader.setUniform4Matrix(simpleMVP, view.GetMVPMatrix());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        wireFrameShader.use();
        cameraMesh->draw();
        simpleShader.use();
        cameraConnectorMesh->draw();

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        cameraMesh->draw();

        currentShaderPointer->use();
      }

      currentShaderPointer->use();
      currentGeometryPointer->draw();
      //handle wireframe rendering
      if (view.wireframe)
      {
        wireFrameShader.setUniform4Matrix(wireFrameMVP, view.GetMVPMatrix());
        wireFrameShader.use();
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        currentGeometryPointer->draw();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        currentShaderPointer->use();
      }
      view.SetViewMatrices(); //calculates mvp matrix. Changes come from setRealCPos and mouse movement.
      glBindFramebuffer(GL_FRAMEBUFFER, 0); //unbind framebuffer

      //######################################## Start ImGUI stuff #######################################################
      //######################################## Start ImGUI stuff #######################################################
      //######################################## Start ImGUI stuff #######################################################
      //######################################## Start ImGUI stuff #######################################################
      //######################################## Start ImGUI stuff #######################################################
      // Start the Dear ImGui frame, no ide what this does internally
      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      //get viewport to make next element fill whole screen
      ImGuiViewport *imgui_viewport = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos(imgui_viewport->GetWorkPos());
      ImGui::SetNextWindowSize(imgui_viewport->GetWorkSize());
      ImGui::SetNextWindowViewport(imgui_viewport->ID);

      //This element fills the whole screen and is a docking space with a menu bar.
      PandiaGui::window_Menubar(view);

#ifdef DEVELOPERTOOLS
      if (PandiaGui::menu_DebugWindow)
      {
        PandiaGui::showDebugWindow(m, view, GUIIteartionTime);
      }
      if (PandiaGui::menu_DemoWindow)
      {
        ImGui::ShowDemoWindow();
      }
#endif //DEVELOPERTOOLS

      //################################## left menu bar static components ############################
      ImGui::Begin("Control Window", NULL, PandiaGui::subWindow_flags | ImGuiWindowFlags_AlwaysVerticalScrollbar);
      PandiaGui::absolutWindowPos = ImGui::GetWindowPos();
      bool oldflag = PandiaGui::loadDataFlag;
      PandiaGui::GuiBlockItem(PandiaGui::disableInput); //GuiBlockItem needs to be set after ImGui-Item which to block and before next ImGui-Item
      ImGui::Checkbox(PandiaGui::lan_loadData.c_str(), &PandiaGui::loadDataFlag);
      threadCom::g_take_dataCam = PandiaGui::loadDataFlag;
      if (oldflag != PandiaGui::loadDataFlag)
      {
        threadCom::g_cameraParameterSet = false;
        threadCom::g_cameraConnected = false;
        if (!PandiaGui::loadDataFlag)
          currentGeometryPointer = &hostMesh;
      }

      PandiaGui::InfoMarker(PandiaGui::lan_loadDataInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
      ImGui::Spacing();
      ImGui::Spacing();
      ImGui::Separator();
      ImGui::Spacing();
      ImGui::Spacing();

      if (PandiaGui::loadDataFlag)
      { //reconstruction from data

        PandiaGui::GuiBlockItem(PandiaGui::disableInput);
        ImGui::RadioButton(PandiaGui::lan_loadMesh.c_str(), &PandiaGui::loadOptionSwitch, 0);
        PandiaGui::InfoMarker(PandiaGui::lan_loadMeshInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);

        ImGui::Spacing();
        ImGui::Spacing();

        PandiaGui::GuiBlockItem(PandiaGui::disableInput);
        ImGui::RadioButton(PandiaGui::lan_loadImages.c_str(), &PandiaGui::loadOptionSwitch, 1);
        // cout << "current " << PandiaGui::loadOptionSwitch << endl;
        PandiaGui::InfoMarker(PandiaGui::lan_loadImagesInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);

        ImGui::Text(" ");
        ImGui::Separator();
        ImGui::Text(" ");
        if (PandiaGui::loadOptionSwitch == 0)
        { //load mesh case
          if (ImGui::Button(PandiaGui::lan_loadMeshDirectory.c_str()))
          {
            ImGui::OpenPopup("Load Mesh"); //not the big button, menu button!
            PandiaGui::file_dialogLoadMesh.current_path = PandiaGui::getTopDirectoryPath(PandiaGui::selectedMeshPath);
          }
          PandiaGui::InfoMarker(PandiaGui::lan_loadMeshDirectoryInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
          ImGui::Spacing();
          ImGui::Spacing();
          ImGui::TextUnformatted(PandiaGui::lan_loadMeshTextHeader.c_str());
          PandiaGui::InfoMarker(PandiaGui::lan_loadMeshTextHeaderInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
          ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_yellow);
          ImGui::TextUnformatted(PandiaGui::selectedMeshPath.c_str());
          ImGui::PopStyleColor();

          ImGui::Spacing();
          ImGui::Spacing();
          ImGui::Separator();
          ImGui::Spacing();
          ImGui::Spacing();

          ImGui::Text(" ");
          ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
          ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_green_1);
          ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_green_2);
          ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_green_3);
          //Load Mesh Button!!!!!
          if (ImGui::Button(PandiaGui::lan_ButtonLoadMesh.c_str(), PandiaGui::buttonsize))
          {
            io::ReadTriangleMesh(PandiaGui::selectedMeshPath, *PandiaGui::loadedMesh);
            hostMesh.mesh_ = PandiaGui::loadedMesh;
            hostMesh.updateGeometry();
          }
          ImGui::PopStyleColor(3);
        }

        if (PandiaGui::loadOptionSwitch == 1)
        { //load image case

          PandiaGui::GuiBlockItem(PandiaGui::disableInput);
          if (ImGui::Button(PandiaGui::lan_loadImagesDirectory.c_str()))
          {
            ImGui::OpenPopup("Load Images From"); //not the button!
            //navigates to one directory above current one and sets FileDialog to custom path
            //to enable this current_path must be changed from private to public
            PandiaGui::file_dialogLoadImage.current_path = PandiaGui::getTopDirectoryPath(g_readimagePath);
          }
          PandiaGui::InfoMarker(PandiaGui::lan_loadImagesDirectoryInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
          ImGui::Spacing();
          ImGui::Spacing();
          ImGui::TextUnformatted(PandiaGui::lan_loadImagesTextHeader.c_str());
          PandiaGui::InfoMarker(PandiaGui::lan_loadImagesTextHeaderInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
          ImGui::PushStyleColor(ImGuiCol_Text, PandiaGui::col_yellow);
          ImGui::TextUnformatted(g_readimagePath.c_str());
          ImGui::PopStyleColor();

          ImGui::Spacing();
          ImGui::Spacing();
          ImGui::Separator();
        }
      }

      PandiaGui::handle_FileDialogs(superscannerSpace, m);

      //################################### now start with programm state dependend gui stuff #########################
      PandiaGui::buttonsize = ImVec2(ImGui::GetWindowSize().x * 0.8f, 60.f);

      switch (threadCom::g_programState)
      {

        //nothing going on yet
      case threadCom::gui_READY:
        //set mouse state dependend colors
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
        if (PandiaGui::loadOptionSwitch == 1 || PandiaGui::loadDataFlag == 0)
        { //dont show button if we are loading mesh

          if (threadCom::g_cameraParameterSet)
          { //enable button
            // if(!m.globalGrid->intrinsicSet){
            //   m.globalGrid->setIntrinsic(g_intrinsic);
            //   cout << "initialized Voxel Grid after receiving camera intrinsic \n";
            // }
            switch (g_camType)
            {
            case camtyp::typ_kinect:
              ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(PandiaGui::lan_InfoAzure.c_str()).x) * 0.5f);
              ImGui::TextUnformatted(PandiaGui::lan_InfoAzure.c_str());
              break;
            case camtyp::typ_realsense:
              ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(PandiaGui::lan_InfoRealsense.c_str()).x) * 0.5f);
              ImGui::TextUnformatted(PandiaGui::lan_InfoRealsense.c_str());
              break;
            case camtyp::typ_data:
              ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(PandiaGui::lan_InfoDataCam.c_str()).x) * 0.5f);
              ImGui::TextUnformatted(PandiaGui::lan_InfoDataCam.c_str());
              break;
            default:
              break;
            }

            ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_green_1);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_green_2);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_green_3);
            //Start Button!
            if (ImGui::Button(PandiaGui::lan_ButtonStart.c_str(), PandiaGui::buttonsize))
            {
              view.setFOVfromCamera(g_intrinsic);
              view.setVirtualIntrinsic();
              reconstructionThread = thread(reconrun, std::ref(m), true); //model, test, livevis, integration
              hostMesh.mesh_ = m.globalGrid->mesh;
              hostMesh.updateGeometry();
              PandiaGui::disableInput = true;
              threadCom::g_programState = threadCom::gui_RUNNING;
            };
            ImGui::PopStyleColor(3);
          }
          else
          { //block button
            sprintf(PandiaGui::animationBuffer, PandiaGui::lan_AnimationText.c_str(), "|/-\\"[(int)(ImGui::GetTime() / 0.25f) & 3]);
            ImGui::SetCursorPosX((ImGui::GetWindowSize().x - ImGui::CalcTextSize(PandiaGui::lan_AnimationText.c_str()).x) * 0.5f);
            ImGui::TextColored(PandiaGui::col_yellow, PandiaGui::animationBuffer, 0);
            ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
            ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_gray);
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_gray);
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_gray);
            ImGui::Button(PandiaGui::lan_ButtonStartDummy.c_str(), PandiaGui::buttonsize);
            ImGui::PopStyleColor(3);
          }
        }
        break;
        //reconrun is executing
      case threadCom::gui_RUNNING:
        //set mouse state dependend colors
        ImGui::Text(" ");
        ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_red_1);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_red_2);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_red_3);
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
        //pause button
        if (ImGui::Button(PandiaGui::lan_ButtonPause.c_str(), PandiaGui::buttonsize))
        {
          //note if something changes here, same must be done in reconrun nread pause segment
          threadCom::g_pause = true;
          threadCom::g_programState = threadCom::gui_PAUSE;
        }
        //GuiBlockItem(g_cameraWarmup, buttonsize);
        ImGui::PopStyleColor(3);

        break;
        //programm paused or reconstruction finished
      case threadCom::gui_PAUSE:
        if (!threadCom::g_current_slam_finished)
        {
          ImGui::Text(" ");
          //color stuff
          ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_green_1);
          ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_green_2);
          ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_green_3);
          ImGui::Spacing();
          ImGui::Spacing();
          ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
          //resume button
          if (ImGui::Button(PandiaGui::lan_ButtonResume.c_str(), PandiaGui::buttonsize))
          {
            threadCom::g_pause = false;
            threadCom::g_programState = threadCom::gui_RUNNING; //programm running again
            PandiaGui::cameraPathBuild = false;
            cleanMemoryTimer.reset();
          }
          PandiaGui::GuiBlockItem(threadCom::g_postProcessing, PandiaGui::buttonsize);
          ImGui::PopStyleColor(3);
        }

        //save
        ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_green_1);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_green_2);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_green_3);
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
        //save mesh button
        if (ImGui::Button(PandiaGui::lan_ButtonSaveMesh.c_str(), PandiaGui::buttonsize))
        {
          PandiaGui::file_dialogSaveMesh.current_path = PandiaGui::getTopDirectoryPath(PandiaGui::saveMeshPath);
          ImGui::OpenPopup("Save mesh to"); //this start the Open File Dialog method
          threadCom::g_programState = threadCom::gui_PAUSE;
        }
        //is true if any buffer has any data
        PandiaGui::nonemptyBuffer = !(pandia_integration::reintegrationBuffer.size() == 0 && pandia_integration::integrationBuffer.size() == 0 && pandia_integration::deintegrationBuffer.size() == 0);
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing, PandiaGui::buttonsize);
        PandiaGui::GuiBlockItem(PandiaGui::nonemptyBuffer, PandiaGui::buttonsize);
        ImGui::PopStyleColor(3);
        //clear mesh
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::PushStyleColor(ImGuiCol_Button, PandiaGui::col_red_1);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, PandiaGui::col_red_2);
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, PandiaGui::col_red_3);
        ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
        //clear mesh button
        if (ImGui::Button(PandiaGui::lan_ButtonClearMesh.c_str(), PandiaGui::buttonsize))
        {
          threadCom::g_clear_button = true; //needs to be set before threads join, thread communication
          threadCom::g_pause = false;       //needs to be set before threads join
          PandiaGui::cameraPathBuild = false;
          if (reconstructionThread.joinable())
          {
            reconstructionThread.join();
          }
          resetAll = true;                   //mark for beginning of loop
          threadCom::g_clear_button = false; //needs to be set after threads have joined
          pandia_integration::stopintegrating = false;
          PandiaGui::disableInput = false;
          //reset postprocessing flags
          PandiaGui::pp_denseAlign = false;
          PandiaGui::pp_meshReduction = false;
          PandiaGui::pp_voxelLength = false;
          //reset frame, chunk ID
          frame_id_counter.id = 0;
          chunk_id_counter.id = 0;
          threadCom::g_programState = threadCom::gui_READY;
        }
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing, PandiaGui::buttonsize);
        ImGui::PopStyleColor(3);
        ImGui::Spacing();

        //################################### save images stuff ###############
#ifdef DEVELOPERTOOLS
        PandiaGui::window_SaveImageTrajectory(m);
#endif //DEVELOPERTOOLS

        //#################################### post processing ########################################################
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextUnformatted("Intermediate Tuning");
        PandiaGui::InfoMarker("After using these functions you can continue your scan.", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
        ImGui::Spacing();
        //################### non linear opt #############################
        ImGui::Spacing();
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
        ImGui::Checkbox("Optimize", &PandiaGui::pp_nonLinearOpt);
        PandiaGui::InfoMarker("Optimze with non linear optimization of keypoints", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
       
        //#######################################################################
        ImGui::Separator();
        ImGui::TextUnformatted("Final Tuning");
        PandiaGui::InfoMarker("After using these functions you cannot continue your scan.", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
        ImGui::Spacing();
        //######################################### mesh reduction ##########################
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
        ImGui::Checkbox(PandiaGui::lan_meshReduction.c_str(), &PandiaGui::pp_meshReduction);
        PandiaGui::InfoMarker(PandiaGui::lan_meshReductionInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
        if (PandiaGui::pp_meshReduction)
        {
          PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
          ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
          ImGui::SliderFloat("##reduction", &PandiaGui::meshSlider, 1.f, 100.f, "%.0f %%");
          ImGui::SameLine();
          ImGui::PushButtonRepeat(true);
          if (ImGui::ArrowButton("decreaseMeshReduction", ImGuiDir_Left) && PandiaGui::meshSlider > 1.f)
          {
            PandiaGui::meshSlider -= 1.f;
            if (PandiaGui::meshSlider < 1.f)
              PandiaGui::meshSlider = 1.f;
          };
          ImGui::SameLine();
          if (ImGui::ArrowButton("increaseMeshReduction", ImGuiDir_Right) && PandiaGui::meshSlider < 100.f)
          {
            PandiaGui::meshSlider += 1.f;
            if (PandiaGui::meshSlider > 100.f)
              PandiaGui::meshSlider = 100.f;
          };
          ImGui::PopButtonRepeat();
        }

        //################### voxel length #############################
        ImGui::Spacing();
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
        ImGui::Checkbox(PandiaGui::lan_voxelLength.c_str(), &PandiaGui::pp_voxelLength);
        PandiaGui::InfoMarker(PandiaGui::lan_voxelLenghtInfo.c_str(), PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);
        if (PandiaGui::pp_voxelLength)
        {
          PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
          ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
          ImGui::SliderFloat("##voxel length", &PandiaGui::voxelSlider, 0.4f, 3.0f, "%.1f cm");
          ImGui::SameLine();
          ImGui::PushButtonRepeat(true);
          if (ImGui::ArrowButton("decreaseVoxelLength", ImGuiDir_Left) && PandiaGui::voxelSlider > 0.5f)
          {
            PandiaGui::voxelSlider -= 0.1f;
            if (PandiaGui::voxelSlider < 0.5f)
              PandiaGui::voxelSlider = 0.5f;
          };
          ImGui::SameLine();
          if (ImGui::ArrowButton("increaseVoxelLength", ImGuiDir_Right) && PandiaGui::voxelSlider < 3.f)
          {
            PandiaGui::voxelSlider += 0.1f;
            if (PandiaGui::voxelSlider > 3.f)
              PandiaGui::voxelSlider = 3.f;
          };
          ImGui::PopButtonRepeat();
        }

        //################### filter taubin #############################
        ImGui::Spacing();
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
        ImGui::Checkbox("Smooth Mesh", &PandiaGui::pp_filterTaubin);
        PandiaGui::InfoMarker("Smooth the Mesh using the Taubin Filter", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);


        //################### dense o3d opt #############################
        ImGui::Spacing();
        PandiaGui::GuiBlockItem(threadCom::g_postProcessing);
        ImGui::Checkbox("Dense ICP Optimization", &PandiaGui::pp_denseo3dOpt);
        PandiaGui::InfoMarker("Optimize densely with ICP.", PandiaGui::menu_InfoMode, PandiaGui::infoSymbol_texture);

        //####################### post processing button ###########################################
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::SetCursorPosX((ImGui::GetWindowSize().x - PandiaGui::buttonsize.x) * 0.5f);
        if (ImGui::Button(PandiaGui::lan_ButtonStartPostProcessing.c_str(), PandiaGui::buttonsize))
        {
          //progress/totalwork makes progress bar.
          PandiaGui::currentProgress = 0.f;
          PandiaGui::totalPPWork = std::numeric_limits<float>::max();
          threadCom::g_postProcessing = true;
          // threadCom::g_pause = false; // let programm resume so that it finishes //todo debug for now
          // if (reconstructionThread.joinable())
          // reconstructionThread.join();
          threadCom::g_programState = threadCom::gui_PAUSE;
          //can only start if at least one post process checkbox is ticked
          postProcessingThread = thread(PostProcessingThreadFunction, std::ref(m));
        }
        //always block if there is no postprocessing
        // PandiaGui::GuiBlockItem(threadCom::g_postProcessing, PandiaGui::buttonsize);

        //if no checkbox is checked we disable the post processing button
        PandiaGui::postpro_disable = !(PandiaGui::pp_denseo3dOpt || PandiaGui::pp_denseAlign || PandiaGui::pp_meshReduction || PandiaGui::pp_voxelLength || PandiaGui::pp_filterTaubin || PandiaGui::pp_nonLinearOpt);
        PandiaGui::GuiBlockItem(PandiaGui::postpro_disable, PandiaGui::buttonsize);
        break;

      default:
        break;
      }
      // ##################### state dependence over #########################

      //Check if any fileDialog is open. Needs to be inside ControlWindow
      PandiaGui::fileDialogOpenNow = (ImGui::IsPopupOpen("Load Mesh") || ImGui::IsPopupOpen("Load Images From") ||
                                      ImGui::IsPopupOpen("Save mesh to") || ImGui::IsPopupOpen("Save Image to:") || ImGui::IsPopupOpen("Save Trajectory to:"));

      ImGui::End();
      //####################################### end element ##########################
      //all the following methods contain a lot of imgui code

      //contains OpenGL Framebuffer and tracking lost indicator
      //MainViewPort for user
      PandiaGui::window_OpenGL(view);

      // PandiaGui::window_bottomInfo();

      //shows indication window when camera is ready to capture picture
      PandiaGui::window_WarmUp();

      //shows progress of postprocessing
      PandiaGui::window_PostProcessing();

      //shows if no camera is connected
      PandiaGui::window_CameraConnection();

      //legend for camera path display
      PandiaGui::window_CameraLegend(m);

      // shows generell information about the firm
      PandiaGui::window_AboutUs();

      //if no longer postprocessing and thread started
      if (!threadCom::g_postProcessing && postProcessingThread.joinable())
      {
        postProcessingThread.join();
        hostMesh.updateGeometry();
        PandiaGui::stopPostProcessing = false;
      }

      // Rendering
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      view.setWindowPos(PandiaGui::topleft.x, PandiaGui::topleft.y);
      if (oldWindowSize.x != PandiaGui::renderWindowSize.x || oldWindowSize.y != PandiaGui::renderWindowSize.y)
        view.ChangeWindowSize(PandiaGui::renderWindowSize.x, PandiaGui::renderWindowSize.y);
      oldWindowSize = PandiaGui::renderWindowSize;

      glfwSwapBuffers(window);
      glfwPollEvents();

    } // end of while loop

    PandiaGui::createPandiaGuiIni();

    //joins Threads if user wants to close program without proper clearing
    threadCom::g_closeProgram = true;
    if (glfwWindowShouldClose(window))
    {
      threadCom::g_clear_button = true; //needs to be set before threads join
      threadCom::g_pause = false;       //needs to be set before threads join
      if (reconstructionThread.joinable())
      {
        reconstructionThread.join();
      }
    }

    cameraConnectionThread.join();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
  } // for deallocating everything kokkos before finalize gets called

  //for deallocation
  pandia_integration::integratedframes.clear(); //doesnt happen otherwise
  pandia_integration::deintegrationBuffer.clear();
  pandia_integration::reintegrationBuffer.clear();
  pandia_integration::integrationBuffer.clear();
  Kokkos::finalize();
  glfwTerminate();
  std::cout << "after everything! \n";

  return 0;
}
//visualization::VisualizerWithCudaModule meshvis;
//if (!meshvis.CreateVisualizerWindow("Live Mesh", 1280, 720, 100, 100)) {
//  utility::LogWarning("Failed creating OpenGL window.\n");
//}
//meshvis.BuildUtilities();
//meshvis.UpdateWindowTitle();
//Timer t;
//meshvis.AddGeometry(m.cudamesh); //takes way long
//t.~Timer();
//meshvis.AddGeometry(getOrigin());
//while (true) {
//  meshvis.PollEvents(); //this takes long (5-20ms) probably blocks a lot!
//  meshvis.UpdateGeometry();
//}
//meshvis.DestroyVisualizerWindow();