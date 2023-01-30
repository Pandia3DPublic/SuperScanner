#include "readconfig.h"
#include "core/threadCom.h"

using namespace std;
void readconfig(std::string s);


//assign to global variables here
template <typename T>
void assignvalue(std::string& s, T& value) {
	if (s == "td") {
		g_td = std::stof(value);
		return;
	}
	if (s == "tc") {
		g_tc = std::stof(value);
		return;
	}
	if (s == "tn") {
		g_tn = std::stof(value);
		return;
	}
	if (s == "nread") {
		g_nread = std::stoi(value);
		return;
	}
	if (s == "nstart") {
		g_nstart = std::stoi(value);
		return;
	}
	if (s == "mergeradius") {
		g_mergeradius = std::stod(value);
		return;
	}
	if (s == "conditionThres") {
		g_conditionThres = std::stod(value);
		return;
	}
	if (s == "minArea") {
		g_minArea = std::stod(value);
		return;
	}
	if (s == "reprojection_threshold") {
		g_reprojection_threshold = std::stod(value);
		return;
	}
	if (s == "nopt") {
		g_nopt = std::stoi(value);
		return;
	}
	if (s == "nkeypoints") {
		g_nkeypoints = std::stoi(value);
		return;
	}
	if (s == "nLocalGroup") {
		g_nLocalGroup = std::stoi(value);
		return;
	}
	if (s == "segment") {
		istringstream(value) >> g_segment;
		return;
	}
	if (s == "readimagePath") {
		istringstream(value) >> g_readimagePath;

		return;
	}
	if (s == "cutoff") {
		g_cutoff = std::stod(value);
		return;
	}
	if (s == "mincutoff") {
		g_mincutoff = std::stod(value);
		return;
	}
	if (s == "voxel_length") {
		g_voxel_length = std::stod(value);
		return;
	}
	if (s == "treint") {
		g_treint = std::stod(value);
		return;
	}

	if (s == "clientdata") {
		g_clientdata = std::stoi(value);
		return;
	}
	if (s == "verbosity") {
		g_verbosity = std::stoi(value);
	}
	if (s == "initial_width") {
		g_initial_width = std::stoi(value);
	}
	if (s == "initial_height") {
		g_initial_height = std::stoi(value);
	}
	if (s == "maxGPUFrames")
	{
		g_maxGPUFrames = std::stoi(value);
	}
}

void readconfig(std::string s) {
	// std::ifstream is RAII, i.e. no need to call close
	cout << "Reading config from " << s << endl;
	std::ifstream cFile(s);
	if (cFile.is_open())
	{
		std::string line;
		while (getline(cFile, line)) {
			line.erase(std::remove_if(line.begin(), line.end(), ::isspace),
				line.end());
			if (line[0] == '#' || line.empty())
				continue;
			auto delimiterPos = line.find("=");
			auto name = line.substr(0, delimiterPos);
			auto value = line.substr(delimiterPos + 1);
			assignvalue(name, value);
		}

	}
	else {
		std::cerr << "Couldn't open config file for reading.\n";
		std::cout << "Working dir is available in c++17 \n" << std::endl;
	}

	if (!checkConfigVariables()) {
		std::cerr << "Not all config variables have been set correctly! \n";
	}
}

bool checkConfigVariables() {

	bool out = true;

	if (g_td == -1) out = false;
	if (g_tc == -1) out = false;
	if (g_tn == -1) out = false;
	if (g_nread == -1) out = false;
	if (g_mergeradius == -1) out = false;
	if (g_conditionThres == -1) out = false;
	if (g_minArea == -1) out = false;
	if (g_reprojection_threshold == -1) out = false;
	if (g_nopt == -1) out = false;
	if (g_nLocalGroup == -1) out = false;
	if (g_nkeypoints == -1) out = false;
	if (g_cutoff == -1) out = false;
	if (g_mincutoff == -1) out = false;
	if (g_verbosity == -1) out = false;
	if (g_voxel_length == -1) out = false;
	if (g_treint == -1) out = false;

	return out;
}

