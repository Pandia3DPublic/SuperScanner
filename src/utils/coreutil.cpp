
#include "coreutil.h"
#include "imageutil.h"
#include "matrixutil.h"
#include "visutil.h"
#include "core/threadCom.h"
using namespace open3d;
using namespace std;

//####################################################with header from here#####################################################################################################################
//these methods assume something about the coordinate system
Eigen::Vector2i pointToImgCoords(const Eigen::Vector3d &p, const Eigen::Matrix3d &intr)
{
	return Eigen::Vector2i(p(0) * intr(0, 0) / p(2) + intr(0, 2), p(1) * intr(1, 1) / p(2) + intr(1, 2));
}

Eigen::Vector3d imgCoordstoPoint(const Eigen::Vector2i &c, float &d, const Eigen::Matrix3d &intr)
{
	return Eigen::Vector3d((c(0) - intr(0, 2)) * d / intr(0, 0), (c(1) - intr(1, 2)) * d / intr(1, 1), d);
}

void prepareDatapath(std::string &s)
{
	for (int i = 0; i < s.length(); i++)
	{
		if (s[i] == '/')
		{
			s.insert(i + 1, "/");
			i++;
		}
	}
}

camera::PinholeCameraIntrinsic getLowIntr(camera::PinholeCameraIntrinsic intrinsic)
{
	Eigen::Matrix3d &intr = intrinsic.intrinsic_matrix_;
	double xfactor = (double)g_lowj / intrinsic.width_;	 // xres
	double yfactor = (double)g_lowi / intrinsic.height_; //yres
	intr(0, 0) *= xfactor;
	intr(0, 2) *= xfactor;
	intr(1, 2) *= yfactor;
	intr(1, 1) *= yfactor;
	intrinsic.width_ = g_lowj;
	intrinsic.height_ = g_lowi;
	return intrinsic;
}

camera::PinholeCameraIntrinsic getScaledIntr(camera::PinholeCameraIntrinsic intrinsic, int width, int height)
{
	Eigen::Matrix3d &intr = intrinsic.intrinsic_matrix_;
	double xfactor = (double)width / intrinsic.width_;	 // xres
	double yfactor = (double)height / intrinsic.height_; //yres
	intr(0, 0) *= xfactor;
	intr(0, 2) *= xfactor;
	intr(1, 2) *= yfactor;
	intr(1, 1) *= yfactor;
	intrinsic.width_ = width;
	intrinsic.height_ = height;
	return intrinsic;
}

//puts the thread to sleep until a frame is available
std::shared_ptr<Frame> getSingleFrame(list<shared_ptr<Frame>> &recordbuffer)
{
	while (threadCom::g_framebuffer.empty() && !threadCom::g_pause)
	{
		std::this_thread::sleep_for(10ms);
	}

	if (threadCom::g_pause)
	{
		return nullptr;
	}

	g_bufferlock.lock();
	auto tmp = threadCom::g_framebuffer.front();
	threadCom::g_framebuffer.pop_front();
#ifdef RECORDBUFFER
	recordbuffer.push_back(tmp);
#endif
	g_bufferlock.unlock();

	return tmp;
}

void setDefaultIntrinsic()
{
	g_intrinsic = camera::PinholeCameraIntrinsic(camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
	g_lowIntr = getLowIntr(g_intrinsic);
	g_fullIntr = g_intrinsic;
}

void checkRecursive(const shared_ptr<Chunk> &c, int indf, vector<int> &indices)
{ //indices contain all traversed nodes
	for (int i = 0; i < c->frames.size(); i++)
	{
		if (indf != i)
		{
			auto t = std::find(indices.begin(), indices.end(), i);
			if (t == indices.end() && c->pairTransforms(c->frames[i], c->frames[indf]).filteredmatches.size() != 0)
			{ // not found
				indices.push_back(i);
				checkRecursive(c, i, indices);
			}
		}
	}
}

//check if the chunk contains frames that are conected with each other, but not with all frames through any number of connections
//check if you can reach any node an the graph from any other node
//todo test this rigorously
bool checkValid(const shared_ptr<Chunk> c, vector<int> &removeIndices)
{
	vector<int> indices;
	indices.reserve(c->frames.size());
	int a = 0;
	indices.push_back(a);
	checkRecursive(c, a, indices);
	bool validchunk = true;
	if (indices.size() < c->frames.size())
	{
		validchunk = false;
		if (indices.size() < c->frames.size() / 2)
		{ //a large unit is missing
			removeIndices = indices;
		}
		else
		{ // a small unit is missing but everything else in removeindex
			for (int i = 0; i < c->frames.size(); i++)
			{ //frame 0 was in the small unit
				auto t = std::find(indices.begin(), indices.end(), i);
				if (t == indices.end())
				{
					removeIndices.push_back(i);
				}
			}
		}
		//delete all filteredmatches
		for (int i = 0; i < removeIndices.size(); i++)
		{
			for (int j = 0; j < c->frames.size(); j++)
			{
				if (removeIndices[i] != j)
				{
					c->pairTransforms(c->frames[removeIndices[i]], c->frames[j]).filteredmatches = vector<match>();
					c->pairTransforms(c->frames[removeIndices[i]], c->frames[j]).set = false;
				}
			}
		}
	}
	//cout << "new indices " << endl;
	//for (int i = 0; i < removeIndices.size(); i++) {
	//	cout << removeIndices[i];
	//}
	return validchunk;
}

void checkRecursiveModel(Model &m, int indf, vector<int> &indices)
{ //indices contain all traversed nodes
	int connections = 0;
	for (int i = 0; i < m.chunks.size(); i++)
	{
		if (indf != i)
		{
			auto t = std::find(indices.begin(), indices.end(), i);
			if (t == indices.end() && m.pairTransforms(m.chunks[i], m.chunks[indf]).filteredmatches.size() != 0) // if not in vector and is valid
			{																									 // not found
				connections++;
				if (connections > 0) //deactivated for now
				{ //todo test this (only push back good node if more than minimum connections found)
					indices.push_back(i);
					checkRecursiveModel(m, i, indices);
				}
			}
		}
	}
}
//check if the model contains chunks that are conected with each other, but not with all chunks through any number of connections
//check if you can reach any node an the graph from any other node
//todo test this rigorously
bool checkValidModel(Model &m, vector<int> &removeIndices)
{
	vector<int> indices; //valid chunk indices
	indices.reserve(m.chunks.size());
	int a = 0;
	indices.push_back(a);
	checkRecursiveModel(m, a, indices);

	// cout << "print indices" << endl;
	// for (auto &i : indices)
	// {
	// 	cout << i << endl;
	// }
	// cout << "sizes" << endl;
	// cout << indices.size() << endl;
	// cout << m.chunks.size() << endl;

	bool validModel = true;
	if (indices.size() < m.chunks.size())
	{
		validModel = false;
		if (indices.size() < m.chunks.size() / 2)
		{ //a large unit is missing
			removeIndices = indices;
		}
		else
		{ // a small unit is missing but everything else in removeindex
			for (int i = 0; i < m.chunks.size(); i++)
			{ //frame 0 was in the small unit
				auto t = std::find(indices.begin(), indices.end(), i);
				if (t == indices.end())
				{
					removeIndices.push_back(i);
				}
			}
		}
		//delete all filteredmatches
		for (int i = 0; i < removeIndices.size(); i++)
		{
			for (int j = 0; j < m.chunks.size(); j++)
			{
				if (removeIndices[i] != j)
				{
					m.pairTransforms(m.chunks[removeIndices[i]], m.chunks[j]).filteredmatches = vector<match>();
					m.pairTransforms(m.chunks[removeIndices[i]], m.chunks[j]).set = false;
				}
			}
		}
	}
	return validModel;
}

//only check the newest chunk here
bool checkValid(Model &m)
{
	bool validChunkInModel = false;
	auto &c = m.chunks.back();
	for (int k = 0; k < m.chunks.size() - 1; k++)
	{
		if (m.pairTransforms(c, m.chunks[k]).set)
		{
			validChunkInModel = true;
		}
	}
	if (!validChunkInModel && m.chunks.size() != 1)
	{
		std::cout << "Chunk number " << m.chunks.size() << " cannot be fitted in model \n";
	}
	return validChunkInModel;
}

// for debug, check if each unit has at least one connection to following unit
bool checkValidStrict(Model &m, std::vector<std::shared_ptr<Frame>> &framesToRemove)
{
	int cc = 0;
	int cf = 0;

	cout << "checking chunks for connections" << endl;
	for (int i = 0; i < m.chunks.size() - 1; i++)
	{
		bool validChunk = false;
		for (int j = i + 1; j < m.chunks.size(); j++)
		{
			if (m.pairTransforms(m.chunks[i], m.chunks[j]).filteredmatches.size() != 0)
			{
				validChunk = true;
			}
		}
		if (!validChunk)
		{
			cout << "#################chunk " << i << " has no connections! \n";
			cc++;
			for (auto &f : m.chunks[i]->frames)
			{
				framesToRemove.push_back(f);
				cf++;
			}
		}
	}

	cout << "checking frames in each chunk for connections" << endl;
	for (int k = 0; k < m.chunks.size(); k++)
	{
		auto &c = m.chunks[k];
		for (int i = 0; i < c->frames.size() - 1; i++)
		{
			bool validFrame = false;
			for (int j = i + 1; j < c->frames.size(); j++)
			{
				if (c->pairTransforms(c->frames[i], c->frames[j]).filteredmatches.size() != 0)
				{
					validFrame = true;
				}
			}
			if (!validFrame)
			{
				cout << "#################chunk " << k << " frame " << i << " has no connections! \n";
				auto t = std::find(framesToRemove.begin(), framesToRemove.end(), c->frames[i]);
				if (t == framesToRemove.end())
				{
					framesToRemove.push_back(c->frames[i]);
					cf++;
				}
			}
		}
	}

	cout << "framesToRemove size " << framesToRemove.size() << endl;
	cout << "remove chunkcount " << cc << endl;
	cout << "remove framecount " << cf << endl;

	if (framesToRemove.size() == 0)
		return true;
	else
		return false;
}

bool getBit(const unsigned char &a, const int &n)
{
	return a >> n & 0x1;
}
void setBitOne(unsigned char &a, const int &n)
{
	a |= 1UL << n;
}
//todo get rid of rgbd and pcd during core algorithm. Note hat rgbd generates a float depth images, which is necessary for
//tsdf integration. Note also that gcf needs pcds.
//This generates a frame from kokkos views. Does not set intrinsics or opencv data. Assumes that images have same size
//this is now on gpu and super fast (<0.5ms)
template <typename SpaceType>
void generateFrame(Kokkos::View<unsigned char **[3]> rgb, Kokkos::View<unsigned short **> depth, shared_ptr<Frame> out, SpaceType &space)
{
	out->rgb = rgb;
	out->depth = depth;
	out->chunktransform = getIdentity();

	//get smaller images with normals for costly calculations
	// Kokkos::View<unsigned char **[3]> rgb_low;
	auto rgb_low = ResizeColorView(space, rgb, g_lowi, g_lowj, true);
	// auto rgbimg = ViewtoImage(rgb_low);
	// visualization::DrawGeometries({rgbimg});

	auto depth_low = ResizeDepthView(space, depth, g_lowi, g_lowj);
	auto low_pcd = CreatePcdFromDepthView(space, depth_low, g_lowIntr);
	auto normals = CalculateNormalsFromPCDView(space, low_pcd);
	auto intensity = CalculateIntensityfromColorView(space, rgb_low);
	// auto rgbi = ViewtoImage(rgb);
	// auto rgbimg = ViewtoImage(rgb_low);
	// auto intensityImg = ViewtoImage(intensity);
	// visualization::DrawGeometries({rgbi});
	// visualization::DrawGeometries({rgbimg});
	// visualization::DrawGeometries({intensityImg});

	// auto pcd = ViewtoO3dPCd(low_pcd);
	// auto img = ViewtoImage(normals);
	// AddViewNormalstoO3DPCD(normals, pcd);
	// cout << pcd->normals_.size() << endl;
	// cout << pcd->points_.size() << endl;
	// visualization::DrawGeometries({img, getOrigin()});
	// visualization::DrawGeometries({pcd, getOrigin()});

	out->lowpcd = low_pcd;
	auto lowpcd_h_LL = Kokkos::create_mirror_view(low_pcd);
	Kokkos::deep_copy(lowpcd_h_LL, low_pcd); //layoutleft on cpu
	Kokkos::View<Kvec3f **, Kokkos::LayoutRight, Kokkos::HostSpace> lowpcd_h("lowpcd_h View", g_lowi, g_lowj);
	Kokkos::deep_copy(lowpcd_h, lowpcd_h_LL); //layout change on cpu
	out->lowpcd_h = lowpcd_h;				  //shallow copy
	out->lowpcd_normals = normals;
	out->lowpcd_intensity = intensity;
	// out->lowpcd_rgb = rgb_low;

	if (!rgb_low.is_allocated() || !depth_low.is_allocated() || !low_pcd.is_allocated() || !normals.is_allocated() || !intensity.is_allocated() || !lowpcd_h_LL.is_allocated())
	{
		cout << "$$$$$$$$$$$$$$$$view not allocated" << endl;
	}
}

template void generateFrame<Kokkos::Cuda>(Kokkos::View<unsigned char **[3]> color, Kokkos::View<unsigned short **> depth, shared_ptr<Frame> out, Kokkos::Cuda &space);

//assigns intrinsic-data from .txt-File
void assignIntrinsic(string value, int lineCounter)
{
	int row = lineCounter - 2;
	if (lineCounter == 0)
		g_intrinsic.width_ = stoi(value);
	if (lineCounter == 1)
		g_intrinsic.height_ = stoi(value);
	if (lineCounter > 1)
	{
		istringstream iss(value);
		for (int column = 0; column < 3; column++)
		{
			double matrixValue;
			iss >> matrixValue;
			g_intrinsic.intrinsic_matrix_(row, column) = matrixValue;
		}
	}
}
//checks if file exists
bool fileExists(const string &filename)
{
	ifstream file;
	file.open(filename);
	if (file)
	{
		return true;
	}
	else
	{
		return false;
	}
}
//reads intrinsic-data from .txt-File
void setFromIntrinsicFile(const string &filepath)
{
	string stringline;
	ifstream cameraIntrinsic(filepath);
	int linecounter = 0;
	if (cameraIntrinsic.is_open())
	{
		while (getline(cameraIntrinsic, stringline))
		{
			assignIntrinsic(stringline, linecounter);
			linecounter++;
		}
		g_lowIntr = getLowIntr(g_intrinsic);
		g_fullIntr = g_intrinsic;
	}
	cameraIntrinsic.close();
}

int CountValidDepthPixelsLocal(shared_ptr<geometry::Image> depth, int stride)
{
	int num_valid_pixels = 0;
	for (int i = 0; i < depth->height_; i += stride)
	{
		for (int j = 0; j < depth->width_; j += stride)
		{
			const uint16_t *p = depth->PointerAt<uint16_t>(j, i);
			if (*p > 0)
				num_valid_pixels += 1;
		}
	}
	return num_valid_pixels;
}

//###############################################################################headeless internal functions#######################################################################################

std::string getPicNumberString(int a)
{
	std::string out = std::to_string(a);
	while (out.length() < 6)
	{
		out = "0" + out;
	}
	return out;
}

//todo get inits over chains of kabsch if it doesnt work with the first frame
//get the initial 6 dofs for optimization for a intrachunk
vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> getDoffromKabschChunk(const shared_ptr<Chunk> c)
{
	vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> out;
	out.push_back(Eigen::Vector6d::Zero()); //first frame in chunk is identity
	for (int i = 1; i < c->frames.size(); i++)
	{
		auto &pt = c->pairTransforms(c->frames[i], c->frames[0]);
		if (pt.set)
		{
			Eigen::Matrix4d tmp = pt.getTransformationFrom(c->frames[i], c->frames[0]);
			out.push_back(MattoDof(tmp));
		}
		else
		{
			out.push_back(Eigen::Vector6d::Zero());
		}
	}

	return out;
}

//todo get inits over chains of kabsch if it doesnt work with the first frame
//is only called if there are at least two chunks!
//uses the chunktransform of last chunk and the kabsch results of the current chunk
vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> getInitialDofs(Model &m)
{
	vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> out;
	out.reserve(m.chunks.size() - 2);
	for (int i = 1; i < m.chunks.size() - 1; i++)
	{
		out.push_back(MattoDof(m.chunks[i]->chunktoworldtrans));
	}
	// last chunk gets transform of former chunk as starting point + kabschtrans
	auto &lastchunk = m.chunks.back();
	auto &pt = lastchunk->pairTransforms(lastchunk->frames.front(), lastchunk->frames.back());
	Eigen::Matrix4d kabscht = Eigen::Matrix4d::Identity();
	if (pt.set)
	{
		Eigen::Matrix4d kabscht = pt.getTransformationFrom(lastchunk->frames.back(), lastchunk->frames.front());
	}

	out.push_back(MattoDof(kabscht * m.chunks[m.chunks.size() - 2]->chunktoworldtrans));

	return out;
}

/*int getdircount(string path) {
	int count = 0;
	for (const auto& entry : std::experimental::filesystem::directory_iterator(path)) {
		count++;
	}
	return count;
}*/

#ifdef RECORDBUFFER
//once cpp 17 replace with std lib functions
void saveImagestoDisc(const string &path, list<shared_ptr<Frame>> &recordbuffer)
{
	cout << "Saving images to " << path << endl;
	cout << "Do not do anything till this is finished !\n";
	//it is guaruanteed that camera paras are set at this point
	ofstream intrinsicFile(path + "intrinsic.txt");
	intrinsicFile << g_intrinsic.width_ << endl
				  << g_intrinsic.height_ << endl
				  << g_intrinsic.intrinsic_matrix_;
	intrinsicFile.close();
	cout << "global intrinsic at saving time was \n"
		 << g_intrinsic.intrinsic_matrix_ << endl;

	int pictureNumber = 0;
	while (!recordbuffer.empty())
	{
		auto tmp = recordbuffer.front();
		recordbuffer.pop_front();
		auto jpgPath = path + "color/" + getPicNumberString(pictureNumber) + ".png";
		auto pngPath = path + "depth/" + getPicNumberString(pictureNumber) + ".png";
		tmp->MovetoGPU();
		io::WriteImage(jpgPath, *ViewtoImage(tmp->rgb));
		auto tmpimg = ViewtoImage(tmp->depth);
		io::WriteImage(pngPath, *tmpimg);
		pictureNumber++;
	}

	cout << "Finished saving images!\n";
}
#endif

#ifdef SAVETRAJECTORY
void saveTrajectorytoDisk(const string &path, Model &m, string name)
{
	cout << "Saving trajectory to " << path << "\n";
	ofstream trajectory(path + name);
	for (int i = 0; i < m.chunks.size(); i++)
	{
		for (int j = 0; j < m.chunks[i]->frames.size(); j++)
		{
			auto &f = m.chunks[i]->frames[j];
			trajectory << f->getFrametoWorldTrans() << endl;
			// trajectory << f->chunktransform << endl;
		}
	}

	trajectory.close();

	cout << "Done saving trajectory \n";
}

void saveChunkTrajectorytoDisk(const string &path, Model &m, string name)
{
	cout << "Saving trajectory to " << path << "\n";
	ofstream trajectory(path + name);
	for (int i = 0; i < m.chunks.size(); i++)
	{
		trajectory << m.chunks[i]->chunktoworldtrans << endl;
	}

	trajectory.close();

	cout << "Done saving chunk trajectory \n";
}
#endif

//reads mat4d as written by saveTrajectorytoDisk
void readTrajectory(const string &path, vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses)
{
	poses.clear();
	cout << "reading " << path << endl;
	ifstream trajectory(path);
	string line;
	std::string delimiter = " ";
	if (trajectory.is_open())
	{
		int lineNumber = 0;
		while (getline(trajectory, line))
		{
			//cout << " line " << line << endl;
			int row = lineNumber % 4;
			if (row == 0)
			{
				poses.emplace_back(Eigen::Matrix4d());
			}
			std::string token;
			size_t pos;
			int col = 0;
			while ((pos = line.find(delimiter)) != std::string::npos)
			{
				token = line.substr(0, pos);
				if (token.compare(""))
				{ //this stupid shit return wrong if strings are equal
					//	cout << token;// << delimiter;
					poses.back()(row, col) = stod(token);
					line.erase(0, pos + delimiter.length());
					col++;
				}
				else
				{
					//cout << "token is empty" << token << "after \n";
					line.erase(0, 1);
				}
			}
			//cout << line <<  endl;

			poses.back()(row, col) = stod(line); //last entry
			lineNumber++;
		}
	}
	else
	{
		cout << "Error: Could not read trajectory file \n";
	}
}
