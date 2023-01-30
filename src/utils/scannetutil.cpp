#define _SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING

#include "scannetutil.h"
#include "jsoncpp/json.h"
#include <experimental/filesystem>

//##################################################### headerless helper functions

std::vector<int> jsonreaderSegs(const std::string& filename) {

	std::string segsName("_vh_clean.segs.json");
	std::ifstream ifs(filename + segsName);
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);
	//std::cout << "sceneId: " << obj["sceneId"].asString() << std::endl;
	std::vector<int> output;
	const Json::Value & segIndicesArray = obj["segIndices"];
	for (int i = 0; i < segIndicesArray.size(); i++) {
		//std::cout << "    segind: " << segIndicesArray[i].asUInt() << std::endl;
		output.push_back(segIndicesArray[i].asUInt());
	}


	return output;
}


void jsonreaderAgg(const std::string& filename, std::vector<std::string>& labels, std::vector<int>& segments) {

	std::string segsName("_vh_clean.aggregation.json");
	std::ifstream ifs(filename + segsName);
	Json::Reader reader;
	Json::Value obj;
	reader.parse(ifs, obj);
	//	std::cout << "sceneId: " << obj["sceneId"].asString() << std::endl;
	std::pair< std::vector<std::string>, std::vector<int>> output;
	const Json::Value & segGroupsArray = obj["segGroups"];
	for (int i = 0; i < segGroupsArray.size(); i++) {
		std::string currentlabel = segGroupsArray[i]["label"].asString();
		//std::cout << "    label: " << segGroupsArray[i]["label"].asString() << std::endl;
		const Json::Value& segmentsArray = segGroupsArray[i]["segments"];
		for (int a = 0; a < segmentsArray.size(); a++) {
			labels.push_back(currentlabel);
			segments.push_back(segmentsArray[a].asUInt());
			//std::cout << "    segments: " << segmentsArray[a].asUInt() << std::endl;
		}
	}
}



void csvtolabels(const std::string& filename, std::vector<std::vector<std::string>>& out) {
	std::string line, word, temp;
	std::ifstream myfile(filename);
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			std::vector<std::string> row;
			//std::cout << line << std::endl;
			std::stringstream s(line);
			while (s.good())
			{
				std::string substr;
				std::getline(s, substr, ',');
				row.push_back(substr);
			}
			auto names = row[5];
			std::stringstream ss(names);
			std::vector<std::string> outvec;
			while (ss.good())
			{
				std::string substr;
				std::getline(ss, substr, ';');
				outvec.push_back(substr);
			}
			out.push_back(outvec);

		}
		myfile.close();

	} else {
		std::cout << "unable to open " << filename << std::endl;
	}
}


void labeltoIdNYU(const std::string& filename, std::vector<std::pair<std::string, uint8_t>>& out) {
	std::string line, word, temp;
	std::ifstream myfile(filename);
	std::pair<std::string, uint8_t> first = std::pair<std::string, uint8_t>("unkown", 39);
	out.push_back(first);
	bool firstit = true;
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			if (firstit) {
				firstit = false;
			} else {
				std::vector<std::string> row;
				//std::cout << line << std::endl;
				std::stringstream s(line);
				while (s.good())
				{
					std::string substr;
					std::getline(s, substr, '\t');
					row.push_back(substr);
				}
				auto raw_category = row[1];
				auto nyu40id = row[4];
				std::pair<std::string, uint8_t> res = std::pair<std::string, uint8_t>(raw_category, std::stoi(nyu40id));
				out.push_back(res);
			}
		}
		myfile.close();

	} else {
		std::cout << "unable to open " << filename << std::endl;
	}
}

Eigen::Matrix4d_u txttoPose(const std::string& filename) {
	std::ifstream myfile(filename);
	Eigen::Matrix4d_u output = Eigen::Matrix4d_u::Zero();
	std::string line, word, temp;
	int x = 0;
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			std::vector<std::string> row;
			std::stringstream s(line);
			while (s.good())
			{
				std::string substr;
				std::getline(s, substr, ' ');
				row.push_back(substr);
			}
			for (int y = 0; y < 4; y++) {
				output(x, y) = std::stod(row[y]);
			}
			x++;
		}
		myfile.close();
	} else {
		std::cout << "unable to open " << filename << std::endl;
	}

	return output;
}

//######################################################## header functions #############################################
std::vector<uint8_t>  pervertexlabelScannet(const std::string& filename) {
	auto segsvec = jsonreaderSegs(filename);

	std::vector<std::string> labels;
	std::vector<int> segments;
	jsonreaderAgg(filename, labels, segments);
	/*for (int i = 0; i < labels.size(); i++ ) {
		std::cout << "    label: " << labels[i] << std::endl;
		std::cout << "    segments: " << segments[i] << std::endl;
	}*/
	std::vector<std::string> labelpervertexstring;
	for (auto seg : segsvec) {
		bool found = false;
		for (int i = 0; i < segments.size(); i++) {
			if (segments[i] == seg) {
				labelpervertexstring.push_back(labels[i]);
				found = true;
				break;
			}
		}
		if (!found) {
			labelpervertexstring.push_back("unknown");
		}
	}
	std::vector<uint8_t> output;
	std::vector<std::vector<std::string>> labeltoid;
	csvtolabels("ade20knames.csv", labeltoid);
	std::vector<std::string> nullter;
	nullter.push_back("unknown");
	labeltoid[0] = nullter;

	/*for (int i = 0; i < labeltoid.size(); i++) {
		std::cout << i << ": ";
		for (auto label : labeltoid[i]) {
			std::cout << label << ", ";
		}
		std::cout << std::endl;
	}*/

	for (auto labelpervertex : labelpervertexstring) {
		bool found = false;
		for (int i = 0; i < labeltoid.size(); i++) {
			for (auto labelname : labeltoid[i]) {
				if (labelname == labelpervertex) {
					found = true;
					output.push_back(i - 1);
					break;
				}
			}
			if (found) {
				break;
			}
		}
		if (!found) {
			output.push_back(255);
		}
	}
	return output;
}


std::vector<uint8_t>  pervertexlabelScannetNYU(const std::string& filename) {
	auto segsvec = jsonreaderSegs(filename);

	std::vector<std::string> labels;
	std::vector<int> segments;
	jsonreaderAgg(filename, labels, segments);
	/*for (int i = 0; i < labels.size(); i++ ) {
		std::cout << "    label: " << labels[i] << std::endl;
		std::cout << "    segments: " << segments[i] << std::endl;
	}*/
	std::vector<std::string> labelpervertexstring;
	for (auto seg : segsvec) {
		bool found = false;
		for (int i = 0; i < segments.size(); i++) {
			if (segments[i] == seg) {
				labelpervertexstring.push_back(labels[i]);
				found = true;
				break;
			}
		}
		if (!found) {
			labelpervertexstring.push_back("unkown"); //wird sp�ter zu otherfurniture
		}
	}
	std::vector<uint8_t> output;
	std::vector<std::pair<std::string, uint8_t>> labeltoid;
	labeltoIdNYU("scannetv2-labels.combined.tsv", labeltoid);

	//for (auto label : labeltoid) {
	//	std::cout << label.first << ", " << (int) label.second << ", " << std::endl;
	//}


	for (auto labelpervertex : labelpervertexstring) {
		bool found = false;
		for (int i = 0; i < labeltoid.size(); i++) {
			if (labeltoid[i].first == labelpervertex) {
				found = true;
				output.push_back(labeltoid[i].second);
				break;
			}
		}
		if (!found) {
			output.push_back(39);
		}
	}
	return output;
}

/*void loadCameraPath(const std::string& path, std::vector<Eigen::Matrix4d_u>& out) {
	int count = 0;
	for (const auto& entry : fs::directory_iterator(path))
		count++;
	for (int i = 0; i < count; i++) {
		out.push_back(txttoPose(path + "\\" + std::to_string(i) + ".txt"));
	}
}
*/
Eigen::Matrix3d loadIntrinsic(const std::string& filename) {
	std::ifstream myfile(filename);
	std::string line, word, temp;
	Eigen::Matrix3d output = Eigen::Matrix3d::Zero();
	int x = 0;
	if (myfile.is_open())
	{
		while (getline(myfile, line) && x < 3)
		{
			std::vector<std::string> row;
			std::stringstream s(line);
			while (s.good())
			{
				std::string substr;
				std::getline(s, substr, ' ');
				row.push_back(substr);
			}
			output(x, 0) = std::stod(row[0]);
			output(x, 1) = std::stod(row[1]);
			output(x, 2) = std::stod(row[2]);
			x++;
		}
		myfile.close();
	} else {
		std::cout << "unable to open " << filename << std::endl;
	}
	return output;
}
std::vector < std::pair<uint8_t, uint8_t>> loadADE20ktoNYULabels(const std::string& filename, std::vector<std::string>& names) {
	std::ifstream myfile(filename);
	std::string line, word, temp;
	std::vector<std::pair<uint8_t, uint8_t>> out;
	for (int i = 0; i <= 39; i++) {
		names.push_back("");
	}

	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			std::vector<std::string> row;
			std::stringstream s(line);
			while (s.good())
			{
				std::string substr;
				std::getline(s, substr, ';');
				row.push_back(substr);
			}


			std::vector<std::string> ids;
			std::stringstream ss(row[0]);
			while (ss.good())
			{
				std::string substr;
				std::getline(ss, substr, ',');
				ids.push_back(substr);
			}

			for (int i = 0; i < ids.size(); i++) {
				out.push_back(std::pair<uint8_t, uint8_t>(std::stoi(ids[i]) - 1, std::stoi(row[1])));
				//	std::cout << "cast id " << ids[i] << " to " << row[1] << " for " << row[2] << std::endl;
			}
			names[std::stoi(row[1])] = row[2];

		}
		myfile.close();

	} else {
		std::cout << "unable to open " << filename << std::endl;
	}
	return out;
}
std::vector<std::string> ade20KNames(const std::string& filename) {
	std::string line, word, temp;
	std::ifstream myfile(filename);
	std::vector<std::string> out;

	for (int i = 0; i < 256; i++) {
		out.push_back("");
	}

	int x = 0;
	if (myfile.is_open())
	{
		while (getline(myfile, line))
		{
			std::vector<std::string> row;
			//std::cout << line << std::endl;
			std::stringstream s(line);
			while (s.good())
			{
				std::string substr;
				std::getline(s, substr, ',');
				row.push_back(substr);
			}
			auto names = row[5];
			std::stringstream ss(names);
			std::vector<std::string> outvec;
			while (ss.good())
			{
				std::string substr;
				std::getline(ss, substr, ';');
				outvec.push_back(substr);
			}
			if (x != 0)
				out[x - 1] = outvec[0];
			x++;
		}
		myfile.close();

	} else {
		std::cout << "unable to open " << filename << std::endl;
	}
	out[255] = "unkown";


	return out;
}


