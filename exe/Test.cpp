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
#include "utils/kokkosUtil.h"
#include "VoxelGrid/kokkosFunctions.h"
#include "time.h"
using namespace std;

//this programm is to be used the following way:
//In most circumstances recorded data exists. In this case just execute trackerEquality()
//if the tracker was changed and new data needs to be generated. Do this by executing
//generateTrackerData()
//

bool comparePoseVectors(const Model &m, const vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses)
{
    bool equal = true;
    int count = 0;
    for (int i = 0; i < m.chunks.size(); i++)
    {
        for (int j = 0; j < m.chunks[i]->frames.size(); j++)
        {
            auto &f = m.chunks[i]->frames[j];
            double diff = (f->getFrametoWorldTrans() - poses[count]).norm();
            if (diff > 1e-5)
            {
                equal = false;
                cout << "Unequal Results \n";
                cout << "Position Number " << count << endl;
                cout << "Difference " << diff << endl;
                cout << f->getFrametoWorldTrans() << endl;
            }
            count++;
        }
    }
    return equal;
}

template <typename SpaceType>
bool trackerEquality(SpaceType &space)
{
    string configpath = "../gitTestData/testconfig.txt";
    readconfig(configpath);
    open3d::utility::SetVerbosityLevel((open3d::utility::VerbosityLevel)g_verbosity);
    Model m;
    m.globalGrid->setIntrinsic(space, g_intrinsic);
    threadCom::g_take_dataCam = true;
    g_camType = camtyp::typ_data;
    auto path = g_readimagePath;
    vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
    bool out = true;
    for (int i = 0; i < 5; i++)
    {
        g_readimagePath = path + "S" + to_string(i + 1) + "/";
        cout << "Reading data from " << g_readimagePath << endl;
        initialiseCamera();
        m.~Model();
        new (&m) Model;
        m.globalGrid->setIntrinsic(space, g_intrinsic);
        reconrun(std::ref(m), false);
        string TestDataPath = "../gitTestData/S" + to_string(i + 1) + ".txt";
        readTrajectory(TestDataPath, poses);
        if (!comparePoseVectors(m, poses))
        {
            out = false;
        }
    }

    pandia_integration::integratedframes.clear(); //doesnt happen otherwise
    pandia_integration::deintegrationBuffer.clear();
    pandia_integration::reintegrationBuffer.clear();
    pandia_integration::integrationBuffer.clear();

    return out;
}
template bool trackerEquality<Kokkos::Cuda>(Kokkos::Cuda &space);

//this function generates text data wich contain camera positions for our test scenes
//In addition to the cam poses the testconfig is saved in the same folder
//Older version of files with the same name are automatically overwritten
template <typename SpaceType>
void generateTrackerData(SpaceType &space)
{
    //run reconrun for 5 scenes in the Testscenes folder
    string configpath = "../gitTestData/testconfig.txt";
    readconfig(configpath);
    open3d::utility::SetVerbosityLevel((open3d::utility::VerbosityLevel)g_verbosity);
    Model m;
    m.globalGrid->setIntrinsic(space, g_intrinsic);
    threadCom::g_take_dataCam = true;
    g_camType = camtyp::typ_data;
    auto path = g_readimagePath;
    for (int i = 0; i < 5; i++)
    {
        g_readimagePath = path + "S" + to_string(i + 1) + "/";
        cout << "Reading data from " << g_readimagePath << endl;
        initialiseCamera();
        m.~Model();
        new (&m) Model;
        m.globalGrid->setIntrinsic(space, g_intrinsic);
        reconrun(std::ref(m), false);
        saveTrajectorytoDisk("../gitTestData/", m, "S" + to_string(i + 1) + ".txt");
    }

    cout << "Finished recording camera position data" << endl;
}
template void generateTrackerData<Kokkos::Cuda>(Kokkos::Cuda &space);


template <typename SpaceType>
bool MatrixVectorTest(SpaceType &space, int iterations = 1000)
{
    using namespace Kokkos;
    bool equal = true;
    for (int it = 0; it < iterations; it++) {
        float mat[4][4]; // for standard type test
        float vec[4];
        // Eigen Test
        Eigen::Matrix4f mat_e;
        Eigen::Vector4f vec_e;
        for (int i = 0; i < 4; i++) {
            float r = (float)rand() / (float)(RAND_MAX/10) - 5.0; //random num between -5.0 and 5.0
            vec_e(i) = r;
            vec[i] = r;
            for (int j = 0; j < 4; j++) {
                r = (float)rand() / (float)(RAND_MAX/10) - 5.0;
                mat_e(i, j) = r;
                mat[i][j] = r;
            }
        }
        Eigen::Vector4f res_e = mat_e * vec_e;

        //standard type test
        //note: result of matrix multiplication with eigen is slightly different from standard c++ due to floating point rounding and precision.
        //moreover the result will also slightly differ whether you are using FPU or SSE units 
        float re[4];
        re[0] = mat[0][0] * vec[0] + mat[0][1] * vec[1] + mat[0][2] * vec[2] + mat[0][3] * vec[3];
        re[1] = mat[1][0] * vec[0] + mat[1][1] * vec[1] + mat[1][2] * vec[2] + mat[1][3] * vec[3];
        re[2] = mat[2][0] * vec[0] + mat[2][1] * vec[1] + mat[2][2] * vec[2] + mat[2][3] * vec[3];
        re[3] = mat[3][0] * vec[0] + mat[3][1] * vec[1] + mat[3][2] * vec[2] + mat[3][3] * vec[3];
        
        // for (int i = 0; i < 4; i++) {
        //     float diff = re[i] - res_e(i);
        //     if (re[i] != res_e(i)) {
        //         cout << "not same!" << endl;
        //     }
        // }

        // Kokkos Test
        auto mat_k = EigentoKmat4f(mat_e);
        auto vec_k = EigentoKvec4f(vec_e);
        Kokkos::View<float[4]> res_k("res_k");

        parallel_for(
        "MatrixVector Test Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {1, 1}), KOKKOS_LAMBDA(int i, int j) {
            Kvec4f res = mat_k() * vec_k();
            res_k(0) = res.x;
            res_k(1) = res.y;
            res_k(2) = res.z;
            res_k(3) = res.w;
        });
        auto res_h = create_mirror_view(res_k);
        deep_copy(res_h, res_k);

        for (int i = 0; i < 4; i++) {
            float diff = abs(res_e(i) - res_h(i));
            if (diff > 5e-5) {
                equal = false;
                cout << "Matrix-Vector result diff: " << diff << endl;
            }
        }
    }
    
    return equal;
}
template bool MatrixVectorTest<Kokkos::Cuda>(Kokkos::Cuda &space, int iterations);



template <typename SpaceType>
bool TransformPCDTest(SpaceType &space)
{
    using namespace Kokkos;
    // Kokkos::View<Kvec4f **> points;
    // std::vector<float> pcd_data;
    // for (int i = 0; i < g_resi*g_resj*4; i++) {
    //     float r = (float)rand() / (float)(RAND_MAX/10) - 5.0; //random num between -5.0 and 5.0
    //     pcd_data.push_back(r);
    // }
	// Kokkos::View<Kvec4f **, Kokkos::LayoutRight, Kokkos::HostSpace> pcd_h(pcd_data.data(), g_resi, g_resj);
    return false;
}
template bool TransformPCDTest<Kokkos::Cuda>(Kokkos::Cuda &space);



int main()
{
    Kokkos::initialize();
    {
        Kokkos::Cuda testSpace;
        srand((unsigned)time(NULL));
        cout.precision(17);

        bool res = MatrixVectorTest(testSpace);
        cout << "The Matrix-Vector Test result is " << res << endl;
        int tt;
        cin >> tt;

        // generateTrackerData(testSpace);
        bool ok = trackerEquality(testSpace);
        cout << "The Tracker Equality is " << ok << endl;

        pandia_integration::integratedframes.clear(); //doesnt happen otherwise
        pandia_integration::deintegrationBuffer.clear();
        pandia_integration::reintegrationBuffer.clear();
        pandia_integration::integrationBuffer.clear();
    }
    Kokkos::finalize();

    return 0;
}