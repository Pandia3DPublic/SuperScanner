executable=$(head -n 1 CurrentExecutable.txt)
path="build"$executable
echo "Building in " $path
cd $path
time ninja
success=$?

#note that these flags slow down our pipeline strongly!
#export OMP_PROC_BIND=spread
#export OMP_PLACES=threads

######### Kokkos profilling tools coming here ##########
#only ever use one tool!
KOKKOS_TOOL_DIR=$HOME/dev/SuperScanner/3rdParty/kokkos-tools #directory, always equal
export KOKKOSP_KERNEL_FILTER=kernelFilters.lst #kernel filter list

#to use with kernel filter chain variables like this
#export KOKKOS_PROFILE_LIBRARY="$KOKKOS_TOOL_DIR/kp_kernel_filter.so;$KOKKOS_TOOL_DIR/kp_kernel_logger.so" #for memory events timeline

#for memory events timeline
# export KOKKOS_PROFILE_LIBRARY=$KOKKOS_TOOL_DIR/kp_memory_events.so 

#does officiallly not support kernel filters but does, time kernels
# export KOKKOS_PROFILE_LIBRARY=$KOKKOS_TOOL_DIR/kp_kernel_timer.so

#does officiallly not support kernel filters, logs starts and stops in command line, including allocation and deallocations
export KOKKOS_PROFILE_LIBRARY=$KOKKOS_TOOL_DIR/kp_kernel_logger.so

if [ $success -eq 0 ]
then
	echo
	echo "Running program."
	./$executable $@
	gedit "$(ls -Art | tail -n 1)" #to open the timing file
else
	echo 
	echo "Build failed."
fi


