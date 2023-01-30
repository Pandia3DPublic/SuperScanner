#include "utils/coreutil.h"
#include "core/threadCom.h"
#include "readconfig.h"
#include "cameras/CameraThreadandInit.h"
#include "core/reconrun.h"
#include "cmakedefines.h"
#include "utils/visutil.h"
#include "core/integrate.h"
#include "postprocessing/postprocessing.h"
#include "Gui/guiutil.h"


int main()
{
	using namespace std;
	using namespace Kokkos;
	cout << "Running Playground \n";
	Kokkos::initialize();
	{
		Kokkos::Cuda playgroundSpace;
		// Eigen::Matrix<double, 3, 4> mat; 
		// mat.setZero();
		// Eigen::Vector4d v(1,2,3,4);
		// cout << mat * v << endl;
		// 	Kmat4f mat;

		// parallel_for(
		// "Test Kernel", MDRangePolicy<Cuda, Rank<2>>(playgroundSpace, {0, 0}, {1, 1}), KOKKOS_LAMBDA(int i, int j) {
		// 	mat.set(Kvec4f(2, 3, -4, 0),
		// 			Kvec4f(11, 8, 7, 0),
		// 			Kvec4f(2, 5, 3, 0),
		// 			Kvec4f(0, 0, 0, 1));
		// 	Kvec3f vec(3,4,2);
		// 	Kvec3f res = mat * vec;
		// 	res.print();

		// });
		// int tt;
		// cin >> tt;


		string configpath = "config.txt";

		readconfig(configpath);
		Model m;

		threadCom::g_take_dataCam = true;
		g_camType = camtyp::typ_data;
		initialiseCamera();
		m.globalGrid->setIntrinsic(playgroundSpace, g_intrinsic);
		//g_readimagePath = imagepath + getSceneName(i);
		// g_readimagePath = "/home/tim/Documents/scene0000_00/";
		cout << g_readimagePath << endl;
		reconrun(std::ref(m), true);
		// open3d::visualization::DrawGeometries({getvisFrusti(m, *m.chunks[112]->frustum)});
		cout << "finished reconrun" << endl;

		threadCom::g_postProcessing = true;
		PandiaGui::pp_denseo3dOpt = true;
		// PandiaGui::pp_nonLinearOpt = true;
		PostProcessingThreadFunction(m);

		// visualizecurrentMatches(m, false);
	
		// m.globalGrid->marchingCubesHost();
		// open3d::visualization::DrawGeometries({(m.globalGrid->mesh)});
		// visualizetsdfModel(m);
		// visualizecurrentMatches(m);
		// visualizecustomskeypoints(m.chunks[0]->orbKeypoints);
		// saveTrajectorytoDisk("/home/tristan/dev/", m);

		pandia_integration::integratedframes.clear(); //doesnt happen otherwise
		pandia_integration::deintegrationBuffer.clear();
		pandia_integration::reintegrationBuffer.clear();
		pandia_integration::integrationBuffer.clear();
		cout << "end of playground, before kokkos finalize" << endl;
	}
	Kokkos::finalize();

	return 0;
};

// int main(){

// 	shared_ptr<Frame> k1 = make_shared<Frame>();
// 	shared_ptr<Frame> k2 = make_shared<Frame>();

// 	frame_pair_hash ph;
// 	cout << ph({k1, k2}) << endl;
// 	cout << ph({k2, k1}) << endl;
// 	cout << "#### \n";
// 	pair_hash_old ph_o;
// 	cout << ph_o<pair<shared_ptr<Frame>,shared_ptr<Frame>>({k1, k2}) << endl;
// 	cout << ph_o < pair<shared_ptr<Frame>, shared_ptr<Frame>>({k2, k1}) << endl;

// 	return 0;

// }
// int main()
// {

// 	using namespace std;
// 	cout << "Running Playground \n";
// 	Kokkos::initialize();
// 	{
// 		// Eigen::Matrix<double, 3, 4> mat; 
// 		// mat.setZero();
// 		// Eigen::Vector4d v(1,2,3,4);
// 		// cout << mat * v << endl;

// 		Kokkos::Cuda playgroundSpace;

// 		string configpath = "config.txt";

// 		readconfig(configpath);
// 		Model m;

// 		threadCom::g_take_dataCam = true;
// 		g_camType = camtyp::typ_data;
// 		initialiseCamera();
// 		m.globalGrid->setIntrinsic(playgroundSpace, g_intrinsic);
// 		//g_readimagePath = imagepath + getSceneName(i);
// 		// g_readimagePath = "/home/tim/Documents/scene0000_00/";
// 		cout << g_readimagePath << endl;
// 		reconrun(std::ref(m), true);
// 		// open3d::visualization::DrawGeometries({getvisFrusti(m, *m.chunks[112]->frustum)});
// 		cout << "finished reconrun" << endl;
	
// 		// m.globalGrid->marchingCubesHost();
// 		// open3d::visualization::DrawGeometries({(m.globalGrid->mesh)});
// 		// visualizetsdfModel(m);
// 		// visualizecurrentMatches(m);
// 		// visualizecustomskeypoints(m.chunks[0]->orbKeypoints);
// 		// saveTrajectorytoDisk("/home/tristan/dev/", m);

// 		// for (int i = 0; i < m.chunks.size(); i++)
// 		// {
// 		// 	for (int j = 0; j < m.chunks[i]->frames.size(); j++)
// 		// 	{
// 		// 		auto &f = m.chunks[i]->frames[j];
// 		// 		cout << "Frame to Chunk Transform \n";
// 		// 		cout << f->chunktransform << endl;
// 		// 		cout << "chunk to world in frame \n";
// 		// 		cout << f->chunktoworldtrans << endl;
// 		// 	}
// 		// 	// saveChunkTrajectorytoDisk("/home/tristan/dev/", m);
// 		// }

// 		// cout << "Chunktransform as saved in chunks \n";
// 		// cout << m.chunks[0]->chunktoworldtrans << endl;
// 		// cout << endl;
// 		// cout << m.chunks[1]->chunktoworldtrans << endl;

// 		pandia_integration::integratedframes.clear(); //doesnt happen otherwise
// 		pandia_integration::deintegrationBuffer.clear();
// 		pandia_integration::reintegrationBuffer.clear();
// 		pandia_integration::integrationBuffer.clear();
// 		cout << "end of playground, before kokkos finalize" << endl;
// 	}
// 	Kokkos::finalize();

// 	return 0;
// }


// #include "core/Frame.h"

// int main(){
// 	using namespace std;

// 	vector<shared_ptr<Frame>> frames;

// 	for (int i=0; i<5 ; i++){
// 		frames.push_back(make_shared<Frame>());
// 	}

// 	// for (auto f: frames){
// 	// 	cout << f->unique_id << endl;
// 	// }

// 	// for (auto it = std::begin(frames); it != std::end(frames); it++) {
// 	// 	cout << (*it)->unique_id << endl;
// 	// }

// 	// for (int i = 0; i < frames.size(); i++) {
// 	// 	cout << frames[i]->unique_id << endl;
// 	// }

// 	return 0;
// }