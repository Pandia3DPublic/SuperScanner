#include "genCors.h"
#include "core/Chunk.h"
using namespace std;

bool matchsort(match& i, match& j) {
	return i.d < j.d;
}
//returns sorted correspondences between two keypointunits
//the query index (p1 in match) alwayss refer to the frame with the lower unique_id
//todo Sollte auf gpu laufen. Sollte kd-tree artige strutktur fuer binaere daten nutzen
// void getCors(std::shared_ptr<KeypointUnit> f1,std::shared_ptr<KeypointUnit> f2, std::vector<match> &out)
void getCors(pairTransform &out)
{

	vector<cv::DMatch> matches;
	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming"); //todo this should be flann-hamming, but no idea how to do that in c++
	matcher->match(out.k1->orbDescriptors, out.k2->orbDescriptors, matches, cv::Mat());
	//translate to custom format
	out.filteredmatches = std::vector<match>(); //start with empty in case stuff is left. Important if two or more frame were removed.
	out.filteredmatches.reserve(matches.size());
	match tmp;
	for (auto m : matches) {
		tmp.indeces << m.queryIdx, m.trainIdx; //get the indeces
		tmp.d = static_cast<double>(m.distance); // get the distance
		tmp.p1 = out.k1->orbKeypoints[m.queryIdx].p;
		tmp.p2 = out.k2->orbKeypoints[m.trainIdx].p;
		out.filteredmatches.push_back(tmp);

	}
	std::sort(out.filteredmatches.begin(), out.filteredmatches.end(), matchsort);
}
