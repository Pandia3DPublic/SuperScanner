#pragma once

//utility to read scannet lables
std::vector<uint8_t>  pervertexlabelScannet(const std::string& filename);
//utility to read scannet lables converted to nyu
std::vector<uint8_t>  pervertexlabelScannetNYU(const std::string& filename);
//load scannet camera path for one scene
//void loadCameraPath(const std::string& path, std::vector<Eigen::Matrix4d_u>& out);
//load scannet intrinisc
Eigen::Matrix3d loadIntrinsic(const std::string& filename);
//load conversion table form ade20k labels to nyu labels from data.
std::vector < std::pair<uint8_t, uint8_t>> loadADE20ktoNYULabels(const std::string& filename, std::vector<std::string>& names);
//load ade20k label names form file. Example file is ade20knames.csv in SegmentationPython
std::vector<std::string> ade20KNames(const std::string& filename);