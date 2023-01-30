#include "Model.h"
#include "utils/matrixutil.h"
#include "utils/coreutil.h"
#include "utils/visutil.h"
#include "Gui/guiutil.h"
#include "core/threadCom.h"
#include "c_keypoint.h"
#include "KeypointOptimizer.h"

using namespace open3d;
using namespace std;

Model::Model()
{
	globalGrid = make_shared<GPUVoxelGrid>(1e8, 0.05, g_voxel_length, g_resi, g_resj);
}

Model::~Model()
{
}

bool Model::allChunkshavePos()
{
	bool allHavePos = true;
	for (int i = 1; i < chunks.size(); i++)
	{
		if (chunks[i]->chunktoworldtrans == Eigen::Matrix4d::Identity())
		{ //todo dangerous in case two pcds are identical
			allHavePos = false;
		}
	}
	return allHavePos;
}



void Model::OptimizeAlignment(const vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> &initx)
{
	//fill the free variables with inital values
	for (int i = 0; i < chunks.size(); i++)
	{
		chunks[i]->x = initx[i];
	}

	//autodiff
	ceres::Problem problem;
	std::vector<std::pair<std::shared_ptr<KeypointUnit>, std::shared_ptr<KeypointUnit>>> matchpointer;
	//addding the residual blocks
	//starting with special case of first block
	for (int i=1 ;i < chunks.size() ; i++){
		auto& c2 = chunks[i];
		auto& tmp = pairTransforms(chunks[0],c2);
		for (int k =0 ;k < tmp.filteredmatches.size() ; k++){
			//automatic
			ceres::CostFunction *costfunction = 
			new ceres::AutoDiffCostFunction<KeypointOptimizerSingleDof, 3, 6>(new KeypointOptimizerSingleDof(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
			matchpointer.push_back(std::make_pair(chunks[0], c2)); // needs to be in this loop so residual count correlates (filteredmatches size varies)
			//numeric
			// ceres::CostFunction *costfunction = 
			// new ceres::NumericDiffCostFunction<ModelOptimizerNumeric, ceres::CENTRAL, 3, 6, 6>(new ModelOptimizerNumeric(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
			problem.AddResidualBlock(costfunction, nullptr, c2->x.data());
		}
	}
	for (int i = 1; i < chunks.size() - 1; i++)
	{
		auto& c1 = chunks[i];
		for (int j = i + 1; j < chunks.size(); j++)
		{
			auto& c2 = chunks[j];
			auto& tmp = pairTransforms(c1,c2);
			for (int k =0 ;k < tmp.filteredmatches.size() ; k++){
				//automatic
				ceres::CostFunction *costfunction = 
				new ceres::AutoDiffCostFunction<KeypointOptimizer, 3, 6, 6>(new KeypointOptimizer(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
				matchpointer.push_back(std::make_pair(c1, c2)); // needs to be in this loop so residual count correlates (filteredmatches size varies)
				//numeric
				// ceres::CostFunction *costfunction = 
				// new ceres::NumericDiffCostFunction<ModelOptimizerNumeric, ceres::CENTRAL, 3, 6, 6>(new ModelOptimizerNumeric(tmp.filteredmatches[k].p1, tmp.filteredmatches[k].p2));
				problem.AddResidualBlock(costfunction, nullptr, c1->x.data(), c2->x.data());
			}
		}
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
	//cout << "Model Successfull steps: " << summary.num_successful_steps << endl;
	//cout << "Model bad steps: " << summary.num_unsuccessful_steps << endl;
	utility::LogDebug("Optimized Model \n");

	// set transformations
	for (int i = 1; i < chunks.size() ; i++)
	{
		chunks[i]->chunktoworldtrans = getT(chunks[i]->x.data());
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
	if (max > 0.05)
	{
		int matchindex = maxindex / 3;
		//remove matches
		pairTransforms.removeElement(matchpointer[matchindex].first, matchpointer[matchindex].second);
		utility::LogInfo("Matches removed by residual filter in Model opt. Biggest residual was {} \n", residuals[maxindex]);
		//take current result as initial values
		vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> tmp;
		for (int i = 0; i < chunks.size(); i++)
		{
			tmp.push_back(chunks[i]->x);
		}
		cout << "high residual filter applied ! " << residuals[maxindex] << endl;;
		OptimizeAlignment(tmp); //use current dof for new staring point
	}
	else
	{
		cout << "highest residual was " << residuals[maxindex] << endl;
	}

}

//only call this in integrationlock!
void Model::setWorldTransforms()
{

	for (int j = 0; j < chunks.size(); j++)
	{
		for (int k = 0; k < chunks[j]->frames.size(); k++)
		{
			auto &f = chunks[j]->frames[k];
			f->setFrametoWorldTrans(chunks[j]->chunktoworldtrans * f->chunktransform);
			f->worldtransset = true;
		}
	}
}


bool Model::doChunktoModelforNewestChunk()
{
	Eigen::Vector3d xt = Eigen::Vector3d::Zero();
	Eigen::Vector3d tsum = Eigen::Vector3d::Zero();
	int count = 0;
	vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>> quats; //quaternions for latter averagging

	auto &c = chunks.back();
	int b = chunks.size() - 1;
	if (b == 0)
		return true;
	for (int a = 0; a < chunks.size() - 1; a++)
	{
		auto &pt = pairTransforms(chunks[a], c);
		if (pt.set)
		{
			if (!chunks[a]->chunktoworldtrans.isIdentity() || a == 0)
			{ //todo first condition superflous?
				count++;
				Eigen::Matrix4d pcb = chunks[a]->chunktoworldtrans * pt.getTransformationFrom(c, chunks[a]);

				quats.push_back(Eigen::Quaterniond(pcb.block<3, 3>(0, 0)));
				tsum = tsum + pcb.block<3, 1>(0, 3);
				// if (b == 112){
				// 	cout << "a " << a << endl;
				// 	cout << &pairTransforms(a, b) << endl;
				// 	cout << "tsum " <<  tsum << endl << endl;
				// }
			}
		}
	}

	if (count > 0)
	{
		xt = tsum.array() / count;
		Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
		out.block<3, 3>(0, 0) = Eigen::Matrix3d(getQuaternionAverage(quats));
		out.block<3, 1>(0, 3) = xt;
		c->chunktoworldtrans = out;
		return true;
	}
	else
	{
		return false;
	}
}


void Model::saveCPUMesh(string name)
{
	auto tmpmesh = *globalGrid->mesh; // copy so that we don't flip multiple times if mesh is saved more than once

	//prevent things from crashing due to out of bounds colors todo check why this is necessary
	for (Eigen::Vector3d &c : tmpmesh.vertex_colors_)
	{
		if (c(0) > 1 || c(0) < 0)
		{
			c(0) = 0;
		}
		if (c(1) > 1 || c(1) < 0)
		{
			c(1) = 0;
		}
		if (c(2) > 1 || c(2) < 0)
		{
			c(2) = 0;
		}
	}
	tmpmesh.Transform(getflip()); //todo avoid having to flip at all
	io::WriteTriangleMesh(name, tmpmesh, false, false, true, true, true);
	cout << "Done saving mesh \n";
}

//stream every frame that is not in the local group to the cpu.
void Model::StreamNonLGtoCPU(vector<shared_ptr<Chunk>> &localGroup)
{
	for (int i = 0; i < chunks.size(); i++)
	{
		if (!(std::find(localGroup.begin(), localGroup.end(), chunks[i]) != localGroup.end()))
		{
			//chunk is not in localGroup and can be streamed out of memory!
			for (int j = 0; j < chunks[i]->frames.size(); j++)
			{
				if (!chunks[i]->frames[j]->duplicate)
				{
					chunks[i]->frames[j]->MovetoCPU();
				}
			}
		}
	}
}
