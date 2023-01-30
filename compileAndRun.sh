executable=$(head -n 1 CurrentExecutable.txt)
path="build"$executable
echo "Building in " $path
cd $path
time ninja
success=$?
if [ $success -eq 0 ]
then
	echo
	echo "Running program."
	./$executable $@
else
	echo "Build failed."
fi
