echo Running Cmake Script
path="build"$1
if [ $# -eq 1 ]; 
then
  echo "Calling Cmake with Argument" $1
  echo "Building inside "$path
  cd $path
  success=$?
  if [ $success -ne 0 ] 
  then
    echo "### Error: No building folder with the name" $path "exists ###"
    exit 0
  fi
  cmake -G Ninja \
    -D EXECUTABLE=$1 \
    -D CMAKE_BUILD_TYPE=RelWithDebInfo \
    -D CMAKE_CXX_EXTENSIONS=OFF \
    -D CMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -D CMAKE_C_COMPILER=$(which gcc-8) \
    -D CMAKE_CXX_COMPILER=$(which g++-8) \
    -D CMAKE_LINKER=$(which lld) \
    -D CMAKE_CXX_FLAGS="-fuse-ld=lld" ..
  cd ..
  echo $1 > CurrentExecutable.txt
  else 
  cd build
  echo "Error: Please supply the name of the Executable you want to build. Options are: StaticScanner, Undistort and CameraCalibration"
fi

