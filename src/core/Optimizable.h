#pragma once
#include "match.h"
#include "EfficientMatch.h"
//base class that contains all variables needed for intra and interchunk alignment

class Optimizable
{
public:
	Optimizable();
	~Optimizable();
	//variables
	// c_umap<std::string, std::vector<match>> filteredmatches; //vector containing a further vector with filtered matches for each pair of frames (kabsch filter).
	// c_umap<std::string, pairTransform> pairTransforms;
	c_umap pairTransforms;


	//debug only
	// c_umap<std::string, std::vector<match>> rawmatches; //vector containing a further vector with matches for each pair of frames. todo superflous, only for debugging
private:

};

