#include "postprocessing.h"
#include "core/threadCom.h"
#include "Gui/guiutil.h"
#include "utils/coreutil.h"
#include "utils/matrixutil.h"
#include "core/integrate.h"
#include "open3d/pipelines/registration/GlobalOptimizationConvergenceCriteria.h"
#include "open3d/pipelines/registration/GlobalOptimization.h"
#include "open3d/pipelines/registration/GlobalOptimizationMethod.h"
#include "open3d/core/EigenConverter.h"
#include "utils/imageutil.h"
#include "kokkosICP.h"
using namespace std;

vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> getInitialDofsPP(Model &m)
{
	vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d>> out;
	for (int i = 0; i < m.chunks.size(); i++)
	{
		out.push_back(MattoDof(m.chunks[i]->chunktoworldtrans));
	}

	return out;
}

//this function integrate everything in integrationbuffer
//updates the world positions
//updates the reintegration buffer
//reintegrate everything in the reintegrationbuffer
//check if chunks are invalid and deintegrates them
template <typename SpaceType>
void reintegratePP(SpaceType &space, Model &m)
{

	while (!pandia_integration::integrationBuffer.empty())
	{
		//reintegrate here
		shared_ptr<Frame> f = pandia_integration::integrationBuffer.front();
		pandia_integration::integrationBuffer.pop_front();

		pandia_integration::tsdfLock.lock();
		m.globalGrid->integrate(space, f, f->getFrametoWorldTrans().cast<float>());
		f->MovetoCPU();
		f->integrateddofs = f->getWorlddofs();
		pandia_integration::tsdfLock.unlock();
	}

	vector<int> removeIndices; //chunks to be removed
	bool isValid = checkValidModel(m, removeIndices);
	if (!isValid)
	{
		cout << "################################################ model is not valid" << endl;
		for (int i = 0; i < removeIndices.size(); i++)
		{
			cout << "chunks to be removed " << removeIndices[i] << endl;
		}
	}

	pandia_integration::integrationlock.lock();
	m.setWorldTransforms();
	pandia_integration::updateReintegrationBuffer();
	pandia_integration::integrationlock.unlock();

	while (!pandia_integration::reintegrationBuffer.empty())
	{
		//reintegrate here
		shared_ptr<Frame> f = pandia_integration::reintegrationBuffer.front();
		pandia_integration::reintegrationBuffer.pop_front();

		pandia_integration::tsdfLock.lock();
		m.globalGrid->deIntegrate(space, f, getT(f->integrateddofs.data()).cast<float>());
		m.globalGrid->integrate(space, f, f->getFrametoWorldTrans().cast<float>());
		f->MovetoCPU();
		f->integrateddofs = f->getWorlddofs();
		pandia_integration::tsdfLock.unlock();
	}

	//deintegrate invalid frames
	for (auto &i : removeIndices)
	{
		auto &c = m.chunks[i];
		for (auto &f : m.chunks[i]->frames)
		{
			pandia_integration::tsdfLock.lock();
			m.globalGrid->deIntegrate(space, f, f->getFrametoWorldTrans().cast<float>());
			pandia_integration::tsdfLock.unlock();

			pandia_integration::integrationlock.lock();
			f->integrateddofs = Eigen::Vector6d::Zero();
			pandia_integration::integratedframes.remove(f);
			pandia_integration::integrationlock.unlock();

			f->MovetoCPU();
		}
	}

	//deintegrate invalid frames
	// while (!framesToRemove.empty()){
	// 	auto f = framesToRemove.back();
	// 	framesToRemove.pop_back();

	// 	pandia_integration::tsdfLock.lock();
	// 	m.globalGrid->deIntegrate(space, f, f->getFrametoWorldTrans().cast<float>());
	// 	pandia_integration::tsdfLock.unlock();

	// 	pandia_integration::integrationlock.lock();
	// 	f->integrateddofs = Eigen::Vector6d::Zero();
	// 	pandia_integration::integratedframes.remove(f);
	// 	pandia_integration::integrationlock.unlock();

	// 	f->MovetoCPU();
	// }
}
template void reintegratePP<Kokkos::Cuda>(Kokkos::Cuda &space, Model &m);

template <typename SpaceType>
void ICPChunkToChunk(SpaceType &space, Model &m)
{
	open3d::pipelines::registration::PoseGraph pg;
	//##########fill nodes
	for (int i = 0; i < m.chunks.size(); i++)
	{
		auto &c = m.chunks[i];
		pg.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(c->chunktoworldtrans));
	}
	// cout << "filled nodes \n";

	//##########fill edges
	for (int i = 0; i < m.chunks.size() - 1; i++)
	{
		auto &c1 = m.chunks[i];
		for (int j = i + 1; j < m.chunks.size(); j++)
		{
			auto &c2 = m.chunks[j];
			auto &pt = m.pairTransforms(c1, c2);
			if (pt.set)
			{
				// cout << "Optimizing between Chunks number " << i << " and " << j << endl;
				float rsme;
				Eigen::Matrix4d ICPTransform = kokkosICP(space, c1, c2, PointToPoint, &rsme);
				pg.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(i, j, ICPTransform));
			}
		}
	}
	// cout << "filled edges \n";
	//solve
	open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria;
	open3d::pipelines::registration::GlobalOptimizationOption option;
	open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt optimization_method;
	open3d::pipelines::registration::GlobalOptimization(pg, optimization_method, criteria, option);
	// cout << "optimzied pg\n";

	//#########set new values
	for (int i = 0; i < m.chunks.size(); i++)
	{
		auto &c = m.chunks[i];
		c->chunktoworldtrans = pg.nodes_[i].pose_;
	}
	// cout << "set values pg\n";
	// reintegratePP(space, m);
	pandia_integration::integrationlock.lock();
	m.setWorldTransforms();
	pandia_integration::updateReintegrationBuffer();
	pandia_integration::integrationlock.unlock();
	cout << "reintegrated chunks\n";
}
template void ICPChunkToChunk<Kokkos::Cuda>(Kokkos::Cuda &space, Model &m);

template <typename SpaceType>
void ICPFrameToFrame(SpaceType &space, Model &m)
{
	// auto &f1 = m.chunks[0]->frames[0];
	// auto &f2 = m.chunks[1]->frames[0];
	// Eigen::Matrix4d transform = kokkosICP(postprocessingSpace, f1,f2);
	Kokkos::Timer ttotal;
	//optimize within chunks
	for (int i = 0; i < m.chunks.size(); i++)
	{
		auto &c = m.chunks[i];
		//build one pose graph for each chunk
		open3d::pipelines::registration::PoseGraph pg;
		Kokkos::Timer tChunkicp;
		for (int j = 0; j < c->frames.size() - 1; j++)
		{
			auto &f1 = c->frames[j];
			// ##########fill nodes
			pg.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(f1->chunktransform));
			// add last frame node
			if (j == c->frames.size() - 2)
			{
				pg.nodes_.push_back(open3d::pipelines::registration::PoseGraphNode(c->frames[j + 1]->chunktransform));
			}
			for (int k = j + 1; k < c->frames.size(); k++)
			{
				auto &f2 = c->frames[k];
				//tranform from f1 to f2
				float rsme;
				Eigen::Matrix4d ICPTransform = kokkosICP(space, f1, f2, PointToPoint, &rsme);
				pg.edges_.push_back(open3d::pipelines::registration::PoseGraphEdge(j, k, ICPTransform));
			}
		}
		cout << "Chunk time " << tChunkicp.seconds() << endl;
		cout << "filled edges \n";
		//solve
		open3d::pipelines::registration::GlobalOptimizationConvergenceCriteria criteria;
		open3d::pipelines::registration::GlobalOptimizationOption option;
		open3d::pipelines::registration::GlobalOptimizationLevenbergMarquardt optimization_method;
		Kokkos::Timer t;
		open3d::pipelines::registration::GlobalOptimization(pg, optimization_method, criteria, option);
		cout << "posegraph time: " << t.seconds() << endl;
		cout << "optimzied pg for chunk " << i << endl;

		for (int j = 0; j < c->frames.size(); j++)
		{
			auto &f = c->frames[j];
			f->chunktransform = pg.nodes_[j].pose_;
		}
	}
	cout << "total icp time: " << ttotal.seconds() << endl;

	// reintegratePP(space, m);
	pandia_integration::integrationlock.lock();
	m.setWorldTransforms();
	pandia_integration::updateReintegrationBuffer();
	pandia_integration::integrationlock.unlock();
	cout << "reintegrated frames\n";
}
template void ICPFrameToFrame<Kokkos::Cuda>(Kokkos::Cuda &space, Model &m);

//todo deintegrate chunks without position!
void PostProcessingThreadFunction(Model &m)
{

	// Kokkos::initialize(); //todo, with this we can produce errors! good for testing!
	Kokkos::Cuda postprocessingSpace = SpaceInstance<Kokkos::Cuda>::create();

	Kokkos::View<unsigned char[3]> rgb;
	auto rgb_h = Kokkos::create_mirror_view(rgb);
	using namespace open3d;

	//meshReduction
	if (PandiaGui::pp_meshReduction)
	{
		int targetsize = m.globalGrid->mesh->triangles_.size() * PandiaGui::meshSlider / 100.0;
		*m.globalGrid->mesh = *m.globalGrid->mesh->SimplifyQuadricDecimation(targetsize, 0.01, 1.0);
		PandiaGui::pp_meshReduction = false;
	}

	//new voxel length
	if (PandiaGui::pp_voxelLength)
	{
		//make new voxel grid in model and reintegrate everything.
		Kokkos::Timer tvl;
		m.globalGrid->~GPUVoxelGrid();
		new (m.globalGrid.get()) GPUVoxelGrid(1e8, 0.05, PandiaGui::voxelSlider / 100.0, g_resi, g_resj);
		m.globalGrid->setIntrinsic(postprocessingSpace, g_intrinsic);
		PandiaGui::totalPPWork = m.chunks.back()->frames.back()->unique_id;
		for (int i = 0; i < m.chunks.size(); i++)
		{
			for (int j = 0; j < m.chunks[i]->frames.size(); j++)
			{
				auto &f = m.chunks[i]->frames[j];
				if (!f->duplicate)
				{
					pandia_integration::integrationBuffer.push_back(f);
					// m.globalGrid->integrate(postprocessingSpace,f,f->getFrametoWorldTrans().cast<float>());
					// cout << "increasing progress to " << PandiaGui::currentProgress << endl;
				}
			}
		}
		// reintegratePP(postprocessingSpace, m);
		pandia_integration::integrationlock.lock();
		m.setWorldTransforms();
		pandia_integration::updateReintegrationBuffer();
		pandia_integration::integrationlock.unlock();

		PandiaGui::pp_voxelLength = false;
	}
	//smoothing
	if (PandiaGui::pp_filterTaubin)
	{
		*m.globalGrid->mesh = *m.globalGrid->mesh->FilterSmoothTaubin(10);
		PandiaGui::pp_filterTaubin = false;
	}

	//non-linear opt
	if (PandiaGui::pp_nonLinearOpt)
	{
		// m.performSparseOptimization(getInitialDofsPP(m));
		m.OptimizeAlignment(getInitialDofsPP(m));
		// reintegratePP(postprocessingSpace, m);
		pandia_integration::integrationlock.lock();
		m.setWorldTransforms();
		pandia_integration::updateReintegrationBuffer();
		pandia_integration::integrationlock.unlock();


		PandiaGui::cameraPathBuild = false;
		PandiaGui::pp_nonLinearOpt = false;
		cout << "non linear opt pp done \n";
	}

	if (PandiaGui::pp_denseo3dOpt)
	{
		ICPChunkToChunk(postprocessingSpace, m);
		ICPFrameToFrame(postprocessingSpace, m);

		PandiaGui::cameraPathBuild = false;
		PandiaGui::pp_denseo3dOpt = false;
		cout << "finished post processing with graph optimization \n";
	}
	//free gui
	threadCom::g_postProcessing = false;
}

shared_ptr<open3d::geometry::TriangleMesh> ColorOptMesh(Model &m)
{

	// std::vector<std::shared_ptr<geometry::RGBDImage>> rgbd_vector;
	// int nchunks = m.chunks.size();
	// open3d::camera::PinholeCameraTrajectory cameraTrajectory;
	// //auto cameraTrajectory = std::make_shared<camera::PinholeCameraTrajectory>();
	// auto tmp = m.mesher.mesh().Download();
	// //m.integrateCPU();

	// //gets all RGBD-Images from frames in chunks and pushes them in vector
	// for (size_t i = 0; i < nchunks; i++) { //goes through every chunk

	// 	auto& current_chunk = m.chunks[i];

	// 	for (size_t j = 0; j < current_chunk->frames.size(); j++) { //goes through every frame in current chunk

	// 		auto& current_frame = current_chunk->frames[j];
	// 		rgbd_vector.push_back(current_frame->rgbd);

	// 		//gets cameraParameters and sets them in CameraTrajectory
	// 		camera::PinholeCameraParameters tmp_parameters;
	// 		tmp_parameters.intrinsic_ = g_intrinsic;
	// 		//set extrinsic to Matrix4d_u
	// 		tmp_parameters.extrinsic_ = current_frame->getFrametoWorldTrans().inverse();

	// 		cameraTrajectory.parameters_.push_back(tmp_parameters);

	// 	}
	// }

	// //auto trimesh = m.tsdf->ExtractTriangleMesh();

	// cout << tmp->triangles_.size() << endl;
	// cout << tmp->vertices_.size() << endl;

	// color_map::ColorMapOptimizationOption option;
	// option.maximum_iteration_ = 100;
	// option.non_rigid_camera_coordinate_ = true;
	// color_map::ColorMapOptimization(*tmp, rgbd_vector, cameraTrajectory, option);

	shared_ptr<open3d::geometry::TriangleMesh> tmp;
	cout << "Color optimization curretnly not supported! \n";
	return tmp;
}
