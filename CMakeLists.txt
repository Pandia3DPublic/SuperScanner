cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)

CMAKE_POLICY(SET CMP0057 NEW)
project(PandiaScanner LANGUAGES CXX CUDA)

# add_definitions(-save-temps) //to get assembler shit

file(GLOB srcfiles 
${PROJECT_SOURCE_DIR}/src/*.h   
${PROJECT_SOURCE_DIR}/src/*.cpp
${PROJECT_SOURCE_DIR}/src/*.cu
${PROJECT_SOURCE_DIR}/src/*/*.h   
${PROJECT_SOURCE_DIR}/src/*/*.cpp
${PROJECT_SOURCE_DIR}/src/*/*.cc
${PROJECT_SOURCE_DIR}/src/*/*.cu
${PROJECT_SOURCE_DIR}/src/cameras/ClientCamera/*.h
${PROJECT_SOURCE_DIR}/src/cameras/ClientCamera/*.cpp
${PROJECT_SOURCE_DIR}/src/Gui/imgui-docking/*.h
${PROJECT_SOURCE_DIR}/src/Gui/imgui-docking/*.cpp
${PROJECT_SOURCE_DIR}/src/Gui/imgui-docking/FileBrowser/*.h
${PROJECT_SOURCE_DIR}/src/Gui/imgui-docking/FileBrowser/*.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/src)

# add_subdirectory(PandiaO3D)
include_directories( SYSTEM
	${PROJECT_SOURCE_DIR}/pchs
)

#need to export this in the open3d cmake to parent scope. stupid new system since 0.8
# link_directories(${CMAKE_PROJECT_NAME} ${O3D_3RDPARTY_INSTALL_PREFIX}/lib)

set(Open3D_DIR ${PROJECT_SOURCE_DIR}/3rdParty/Open3D/lib/cmake/Open3D)
find_package(Open3D HINTS Open3D_DIR REQUIRED)

set(Kokkos_DIR ${PROJECT_SOURCE_DIR}/3rdParty/kokkos/lib/cmake/Kokkos/)
find_package(Kokkos REQUIRED)

set(CeresEigenHint ${PROJECT_SOURCE_DIR}/3rdParty/Eigen/share/eigen3/cmake)
set(Ceres_DIR ${PROJECT_SOURCE_DIR}/3rdParty/ceres/lib/cmake/Ceres/)
find_package(Ceres REQUIRED)

set(OpenCV_DIR ${PROJECT_SOURCE_DIR}/3rdParty/opencv/lib/cmake/opencv4/)
find_package(OpenCV REQUIRED)

find_package(k4a REQUIRED)

# compile times double for every freaking target!!!!!!!
if (${EXECUTABLE} STREQUAL "Playground")
    message("################### Building Playground ###################")
    set(target_name Playground) #cpp files with same name must be located in /exe 
endif()
if (${EXECUTABLE} STREQUAL "SuperScanner")
    message("################### Building SuperScanner ###################")
    set(target_name SuperScanner) #cpp files with same name must be located in /exe 
endif()
if (${EXECUTABLE} STREQUAL "test")
    message("################### Building Test ###################")
    set(target_name Test) #cpp files with same name must be located in /exe 
endif()



#set(compileDefs DEVELOPERTOOLS EIGEN_DONT_VECTORIZE EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
set(compileDefs ${compileDefs} DEVELOPERTOOLS ENABLEASSERTIONS)
# set(compileDefs DEVELOPERTOOLS)


add_executable(${target_name} exe/${target_name}.cpp ${srcfiles})
target_precompile_headers(${target_name} PUBLIC ${PROJECT_SOURCE_DIR}/pchs/pch.h)
target_link_libraries(${target_name} Kokkos::kokkos)
target_link_libraries(${target_name} ${Open3D_LIBRARIES})
target_include_directories(${target_name} PUBLIC ${Open3D_INCLUDE_DIRS})
target_include_directories(${target_name} PUBLIC ${CERES_INCLUDE_DIRS})
target_link_libraries(${target_name} ${CERES_LIBRARIES})
target_include_directories(${target_name} PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${target_name} ${OpenCV_LIBRARIES})
target_link_libraries(${target_name} k4a)
target_compile_definitions(${target_name} PRIVATE ${compileDefs})

#   add_custom_command(TARGET ${target_name}
#                  POST_BUILD
#                  COMMAND make ARGS ${SOURCE_FILENAME}.s
#                  COMMAND ${CMAKE_COMMAND} -E copy
#                      "${CMAKE_BINARY_DIR}/CMakeFiles/${TARGET}.dir/${SOURCE_FILENAME}.cpp.s"
#                      "${OUTPUT_PATH}/${ASSEMBLY_LISTING_FILE}"
#                  WORKING_DIRECTORY ${CMAKE_BINARY_DIR})



#copy jpeg lib. for linux only
# file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdparty_install)
# file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdparty_install/include)
# file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdparty_install/lib)

# file(GLOB jpegfilesInclude ${PROJECT_SOURCE_DIR}/turbojpeg_libs/include/*)
# foreach( jpegfile ${jpegfilesInclude} )
# configure_file(${jpegfile} ${CMAKE_CURRENT_BINARY_DIR}/3rdparty_install/include COPYONLY )
# endforeach( jpegfile )

# file(GLOB jpegfilesLib ${PROJECT_SOURCE_DIR}/turbojpeg_libs/lib/*.a)
# foreach( jpegfile ${jpegfilesLib} )
# configure_file(${jpegfile} ${CMAKE_CURRENT_BINARY_DIR}/3rdparty_install/lib COPYONLY )
# endforeach( jpegfile )



#copy resources into build folder!
#copy shaders 
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_PATH})
file(GLOB shaders ${PROJECT_SOURCE_DIR}/Resources/shader_outside/*.txt)
foreach( shader ${shaders} )
configure_file(${shader} ${CMAKE_CURRENT_BINARY_DIR}/${SHADER_PATH} COPYONLY )
endforeach( shader )

#copy textures 
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/textures)
file(GLOB textures ${PROJECT_SOURCE_DIR}/Resources/textures_outside/*.png)
foreach( texture ${textures} )
configure_file(${texture} ${CMAKE_CURRENT_BINARY_DIR}/textures COPYONLY )
endforeach( texture )

#copy the config.txt
configure_file(config.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY )
# configure_file(testconfig.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY )


#copy language files 
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/languageFolder)
file(GLOB languageFiles ${PROJECT_SOURCE_DIR}/Resources/language_outside/*.txt)
foreach( languageFile ${languageFiles} )
configure_file(${languageFile} ${CMAKE_CURRENT_BINARY_DIR}/languageFolder COPYONLY )
endforeach( languageFile )

#for kokkos tools, kernel filter file
configure_file(kernelFilters.lst ${CMAKE_CURRENT_BINARY_DIR} COPYONLY )

