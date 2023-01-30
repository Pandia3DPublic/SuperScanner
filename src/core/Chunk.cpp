#include "Chunk.h"
#include "utils/matrixutil.h"
#include "utils/coreutil.h"
#include "core/threadCom.h"
#include "KeypointOptimizer.h"

using namespace open3d;
using namespace std;

Chunk::Chunk()
{
	unique_id = chunk_id_counter;
	isChunk = 1;
}

Chunk::~Chunk()
{
}

//todo make better and faster
void Chunk::generateFrustum()
{
	double minz = 2 * g_cutoff;
	double maxz = 0;
	// for (auto &p : frames[0]->lowpcd->points_)
	// {
	// 	if (p(2) > maxz)
	// 		maxz = p(2);
	// 	if (p(2) < minz)
	// 		minz = p(2);
	// }
	for (int i = 0; i < frames[0]->lowpcd_h.extent(0); i++)
	{
		for (int j = 0; j < frames[0]->lowpcd_h.extent(1); j++)
		{
			if (frames[0]->lowpcd_h(i, j).z != 0)
			{
				if (frames[0]->lowpcd_h(i, j).z > maxz)
					maxz = frames[0]->lowpcd_h(i, j).z;
				if (frames[0]->lowpcd_h(i, j).z < minz)
					minz = frames[0]->lowpcd_h(i, j).z;
			}
		}
	}
	#ifdef ENABLEASSERTIONS
	if (minz ==0){
		cout << "warning frustums dont work for some reason with minz=0" << endl;
	}
	#endif

	frustum = make_shared<Frustum>(Eigen::Matrix4d::Identity(), g_intrinsic, minz, maxz);
}
//uses chunktransform and frametochunktransform to set frametoworldtransform
void Chunk::setWorldTransforms()
{
	for (int k = 0; k < frames.size(); k++)
	{
		auto &f = frames[k];
		f->setFrametoWorldTrans(chunktoworldtrans * f->chunktransform);
		f->worldtransset = true;
	}
}


bool Chunk::doFrametoModelforNewestFrame()
{
	Eigen::VectorXd xt = Eigen::Vector3d::Zero();
	Eigen::VectorXd tsum = Eigen::Vector3d::Zero();
	int count = 0;
	vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> quats; //quaternions for latter averagging

	auto &f = frames.back();
	if (frames.size() == 1)
		return true;
	for (int a = 0; a < frames.size() - 1; a++)
	{
		auto& pt = pairTransforms(frames[a], f);
		if (pt.set)
		{
			if (!frames[a]->chunktransform.isIdentity() || a == 0)
			{
				count++;
				Eigen::Matrix4d pcb = frames[a]->chunktransform * pt.getTransformationFrom(f, frames[a]); 
				quats.push_back(Eigen::Quaterniond(pcb.block<3, 3>(0, 0)));
				tsum = tsum + pcb.block<3, 1>(0, 3);
			}
		}
	}

	if (count > 0)
	{
		xt = tsum / count;
		Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
		out.block<3, 3>(0, 0) = Eigen::Matrix3d(getQuaternionAverage(quats));
		out.block<3, 1>(0, 3) = xt;
		//does not need lock since frame cannot be in integrationbuffer here
		f->chunktransform = out;
		// cout << "out " << out << endl;
		// cout << "chunktrans inside " << f->chunktransform << endl;

		return true;
	}
	else
	{
		return false;
	}
}



bool Chunk::OptimizeAlignment(const vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &initx, bool recursiveCall)
{
	//fill the free variables with inital values
	for (int i = 0; i < frames.size(); i++)
	{
		frames[i]->x = initx[i];
	}

	//autodiff
	ceres::Problem problem;
	std::vector<std::pair<std::shared_ptr<KeypointUnit>, std::shared_ptr<KeypointUnit>>> matchpointer;
	//addding the residual blocks
	//starting with special case of first block
	int nResiduals = 0;
	for (int i=1 ;i < frames.size() ; i++){
		auto& f2 = frames[i];
		auto& tmp = pairTransforms(frames[0],f2);
		for (int k =0 ;k < tmp.filteredmatches.size() ; k++){
			//automatic
			ceres::CostFunction *costfunction = 
			new ceres::AutoDiffCostFunction<KeypointOptimizerSingleDof, 3, 6>(new KeypointOptimizerSingleDof(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
			// matchpointer.push_back(&tmp); // needs to be in this loop so residual count correlates (filteredmatches size varies)
			matchpointer.push_back(std::make_pair(frames[0], f2)); // needs to be in this loop so residual count correlates (filteredmatches size varies)
			//numeric
			// ceres::CostFunction *costfunction = 
			// new ceres::NumericDiffCostFunction<ModelOptimizerNumeric, ceres::CENTRAL, 3, 6, 6>(new ModelOptimizerNumeric(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
			problem.AddResidualBlock(costfunction, nullptr, f2->x.data());
			nResiduals++;
		}
	}
	for (int i = 1; i < frames.size() - 1; i++)
	{
		auto& f1 = frames[i];
		for (int j = i + 1; j < frames.size(); j++)
		{
			auto& f2 = frames[j];
			auto& tmp = pairTransforms(f1,f2);
			for (int k =0 ;k < tmp.filteredmatches.size() ; k++){
				//automatic
				ceres::CostFunction *costfunction = 
				new ceres::AutoDiffCostFunction<KeypointOptimizer, 3, 6, 6>(new KeypointOptimizer(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
				// matchpointer.push_back(&tmp); // needs to be in this loop so residual count correlates (filteredmatches size varies)
				matchpointer.push_back(std::make_pair(f1,f2)); // needs to be in this loop so residual count correlates (filteredmatches size varies)
				//numeric
				// ceres::CostFunction *costfunction = 
				// new ceres::NumericDiffCostFunction<ModelOptimizerNumeric, ceres::CENTRAL, 3, 6, 6>(new ModelOptimizerNumeric(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
				problem.AddResidualBlock(costfunction, nullptr, f1->x.data(), f2->x.data());
				nResiduals++;
			}
		}
	}

	if (nResiduals == 0 ) {
		return true;
	}

	ceres::Solver::Options options;
	options.max_num_iterations = 15;
	//options.linear_solver_type = ceres::DENSE_QR;
	options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //good
	// options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY; //not working
	options.minimizer_progress_to_stdout = false;
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT; // is default
	//options.num_threads = 8; // does apparently nothing, makes it slightly slower
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	// std::cout << summary.FullReport() << "\n";
	//cout << "Chunk Successfull steps: " << summary.num_successful_steps << endl;
	//cout << "Chunk bad steps: " << summary.num_unsuccessful_steps << endl;
	utility::LogDebug("Optimized Chunk \n");

	// set transformations
	for (int i = 1; i < frames.size() ; i++)
	{
		frames[i]->chunktransform = getT(frames[i]->x.data());
	}

	vector<double> residuals;
	double cost;
	problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals, NULL, NULL);
	int maxindex = 0;
	double max = 0;
	for (int i = 0; i < residuals.size(); i++)
	{
		if (fabs(residuals[i]) > max)
		{
			max = fabs(residuals[i]);
			maxindex = i;
		}
	}
	if (fabs(residuals[maxindex]) > 0.05)
	{
		int matchindex = maxindex / 3;
		//remove pairtransform
		pairTransforms.removeElement(matchpointer[matchindex].first, matchpointer[matchindex].second);
		cout << "Matches removed by residual filter in Chunk opt. Biggest residual was " << residuals[maxindex] << endl;
		//take current result as initial values
		vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> tmp;
		for (int i = 0; i < frames.size(); i++)
		{
			tmp.push_back(frames[i]->x);
		}
		OptimizeAlignment(tmp,true); //use current dof for new staring point
	}
	else
	{
		utility::LogDebug("highest residual in chunk was {} \n", residuals[maxindex]);
	}

	return recursiveCall;
}



//this works as inteded.
void Chunk::getMajorityDescriptor(const vector<int> &indeces, const vector<c_keypoint> &kps, vector<uint8_t> &merged)
{
	bool done = false;
	merged.reserve(g_nOrb);
	for (int i = 0; i < g_nOrb; i++)
	{
		merged.push_back(0);
	}

	bool tmp;
	for (int i = 0; i < g_nOrb; i++)
	{
		for (int j = 0; j < 8; j++)
		{
			int count = 0;
			for (auto &ind : indeces)
			{
				tmp = getBit(kps[ind].des[i], j);
				count += tmp; //increment if true
			}
			if (count > indeces.size() / 2)
			{ //this defaults to zero, if we have two points merging
				setBitOne(merged[i], j);
			}
		}
	}

	// test if each descriptor has at least 80% congruence with merged point. Otherwise remove descriptor and start over.
	for (int k = 0; k < indeces.size(); k++)
	{
		int same = 0;
		for (int i = 0; i < g_nOrb; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				if (getBit(kps[indeces[k]].des[i], j) == getBit(merged[i], j))
				{
					same++;
				}
			}
		}
		if (same <= 8 * g_nOrb * 0.8)
		{ // bad point merge, just take orig point
			merged = kps[indeces[0]].des;
		}
	}
}

//only call this after generating validkeypoints
//todo evaluate iterative behaviour and radius size
//todo currently not working correctly for iterative behaviour.
// Add oldweights to newweights when melding points.
// only meld descriptors when successfull
// take weights into account in majority vote
//for now its the non-recursive version, which has been visualy tested
//this takes a little under 1 ms
void Chunk::generateChunkKeypoints()
{
	if (chunktransapplied == false)
	{
		for (int i = 0; i < frames.size(); i++)
		{
			for (int j = 0; j < frames[i]->efficientKeypoints.size(); j++)
			{
				frames[i]->efficientKeypoints[j].transform(frames[i]->chunktransform);
			}
		}
		chunktransapplied = true;
	}

	// if its the first iteration fill keypoints with all frame points
	int n = 0;
	std::vector<c_keypoint> tmpKeypoints;
	for (int i = 0; i < frames.size(); i++)
	{
		n += frames[i]->efficientKeypoints.size();
	}
	tmpKeypoints.reserve(n); //this is a chunk variable, need to do this instead of just build pcd for recursive behaviour.
	for (int i = 0; i < frames.size(); i++)
	{
		for (auto &k : frames[i]->efficientKeypoints)
		{
			tmpKeypoints.push_back(k);
		}
	}
	//build pcd from keypoints for kdtree search
	auto pcd = make_shared<geometry::PointCloud>();
	pcd->points_.reserve(tmpKeypoints.size());
	for (int i = 0; i < tmpKeypoints.size(); i++)
	{
		pcd->points_.push_back(tmpKeypoints[i].p.block<3, 1>(0, 0));
	}

	geometry::KDTreeFlann kdtree;
	kdtree.SetGeometry(*pcd);
	std::vector<int> ignore;
	int k;
	std::vector<c_keypoint> newkeypoints;
	for (int i = 0; i < pcd->points_.size(); i++)
	{
		std::vector<int> indices_vec;
		std::vector<double> dists_vec;
		auto t = std::find(ignore.begin(), ignore.end(), i);
		if (t == ignore.end())
		{ //check if index is on the ignore list. Point is not on list
			Eigen::Vector3d &p = pcd->points_[i];
			k = kdtree.SearchRadius(p, g_mergeradius, indices_vec, dists_vec);
			if (indices_vec.size() == 1)
			{
				newkeypoints.push_back(tmpKeypoints[i]); // push back the point that was checked
			}
			else
			{
				Eigen::Vector3d middle(0, 0, 0);
				int weightsum = 0;
				for (int j = 0; j < indices_vec.size(); j++)
				{
					middle += pcd->points_[indices_vec[j]];
					weightsum += 1;
					ignore.push_back(indices_vec[j]);
				}
				middle /= weightsum;
				c_keypoint tmpk;
				tmpk.p = Eigen::Vector4d(middle(0), middle(1), middle(2), 1.0);
				//tmpk.des = keypoints[i].des; //todo do  majority vote here
				getMajorityDescriptor(indices_vec, tmpKeypoints, tmpk.des);
				newkeypoints.push_back(tmpk);
			}
		}
	}

	orbKeypoints = newkeypoints;
	//success finish method
	//build opencv descriptor from kps
	orbDescriptors = cv::Mat::zeros(orbKeypoints.size(), g_nOrb, CV_8U);
	for (int i = 0; i < orbKeypoints.size(); i++)
	{
		// build the filtered descriptor matrix
		for (int j = 0; j < g_nOrb; j++)
		{
			orbDescriptors.at<uchar>(i, j) = orbKeypoints[i].des[j];
		}
	}
	// cout << orbKeypoints.size() << endl;
}


//we only use this to generate chunk keypoints. We could just use pairtransforms 
//for that since generateChunkKeypoints merges anyway.
void Chunk::generateEfficientKeypoints()
{
	//empty variables
	for (int i = 0; i < frames.size(); i++)
	{
		frames[i]->efficientKeypoints = vector<c_keypoint>();
	}
	// build structures for efficient sparse alignment todo make it faster(not important, minor)
	//efficientkeypoints contains all valid keypoints per frame and not more
	//efficientmatches contains only the matches of these keypoints
	for (int i = 0; i < frames.size() - 1; i++)
	{
		auto f1 = frames[i];
		for (int j = i + 1; j < frames.size(); j++)
		{
			auto f2 = frames[j];
			auto &f = pairTransforms(f1, f2).filteredmatches;
			shared_ptr<Frame> f1f = static_pointer_cast<Frame>(pairTransforms(f1, f2).k1);
			shared_ptr<Frame> f2f = static_pointer_cast<Frame>(pairTransforms(f1, f2).k2);
			for (int k = 0; k < f.size(); k++)
			{
				// cout << "test thingy " << (f1->unique_id > f2->unique_id) << " " << f1->unique_id << " " << f2->unique_id << endl;
				auto candidate = f1f->orbKeypoints[f[k].indeces(0)]; //index is selected by boolean
				auto t = std::find(f1f->efficientKeypoints.begin(), f1f->efficientKeypoints.end(), candidate);
				if (t == f1f->efficientKeypoints.end())
				{
					f1f->efficientKeypoints.push_back(candidate);
				}
				candidate = f2f->orbKeypoints[f[k].indeces(1)];
				t = std::find(f2f->efficientKeypoints.begin(), f2f->efficientKeypoints.end(), candidate);
				if (t == f2f->efficientKeypoints.end())
				{
					f2f->efficientKeypoints.push_back(candidate);
				}
			}
		}
	}
}

void Chunk::deleteStuff()
{
	//todo
}
