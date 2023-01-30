#include "kokkosICP.h"
#include "utils/imageutil.h"
#include "core/threadCom.h"
#include "utils/kokkosUtil.h"
#include "utils/visutil.h"
using namespace std;

template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, shared_ptr<Frame> f1, shared_ptr<Frame> f2, ICPTypes ICPType, float *rsme)
{
    cout << "ICP between frames " << f1->unique_id << " and " << f2->unique_id;
    return kokkosICP(space, f1->depth, f2->depth, f1->getFrametoWorldTrans(), f2->getFrametoWorldTrans(), ICPType, rsme);
}
template Eigen::Matrix4d kokkosICP<Kokkos::Cuda>(Kokkos::Cuda &space, shared_ptr<Frame> f1, shared_ptr<Frame> f2, ICPTypes ICPType, float *rsme);

template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, shared_ptr<Chunk> c1, shared_ptr<Chunk> c2, ICPTypes ICPType, float *rsme)
{
    cout << "ICP between chunk " << c1->unique_id << " and " << c2->unique_id;
    return kokkosICP(space, c1->frames[0]->depth, c2->frames[0]->depth, c1->chunktoworldtrans, c2->chunktoworldtrans, ICPType, rsme);
}
template Eigen::Matrix4d kokkosICP<Kokkos::Cuda>(Kokkos::Cuda &space, shared_ptr<Chunk> c1, shared_ptr<Chunk> c2, ICPTypes ICPType, float *rsme);

template <typename SpaceType>
Eigen::Matrix4d kokkosICP(SpaceType &space, Kokkos::View<unsigned short **> d1, Kokkos::View<unsigned short **> d2,
                          const Eigen::Matrix4d &tx, const Eigen::Matrix4d &ty, ICPTypes ICPType, float *rsme_extern)
{
    using namespace Kokkos;
    if (d1.extent(0) != d2.extent(0) || d1.extent(1) != d2.extent(1))
    {
        cout << "\n In Kokkos ICP unequally sized depth images! \n";
    }
    auto x = CreatePcdFromDepthView(space, d1, g_intrinsic);
    auto y = CreatePcdFromDepthView(space, d2, g_intrinsic);
    auto y_normals = CalculateNormalsFromPCDView(space, y);
    auto x_mirror = create_mirror_view(x);
    deep_copy(x_mirror, x);
    auto y_mirror = create_mirror_view(y);
    deep_copy(y_mirror, y);

    //contains nearest point of y in x
    View<Kvec3f **> x_nearest("Nearest Points View", d1.extent(0), d1.extent(1));

    //local intrinsic parameter
    int resi = x.extent(0);
    int resj = x.extent(1);
    //look at coordiante system drawings for this to make sense.
    float ci = resi - g_intrinsic.intrinsic_matrix_(1, 2);
    float cj = g_intrinsic.intrinsic_matrix_(0, 2);
    float fx = g_intrinsic.intrinsic_matrix_(0, 0);
    float fy = g_intrinsic.intrinsic_matrix_(1, 1);

    int n_search = 5; //search for 10 pixel in every direction
    int n_it = 10;
    float max_dist = 0.075; //7.5cm max point distance
    //transformation from pcd x to pcd y
    Eigen::Matrix4d Txy_eigen = ty.inverse() * tx;
    auto Txy = EigentoKmat4f(Txy_eigen.cast<float>());
    float lowestRSME = 1e6;
    float bestFitness = 0;
    Eigen::Matrix4d bestTxy_eigen = Eigen::Matrix4d::Identity();
    //todo improve: We prob don't need all correspondences. Take less cors with the same dense images
    for (int iteration = 0; iteration < n_it; iteration++)
    {

        //The job of this kernel is to find the closest point for any point in x in y.
        //To do this we transform the z-vector with the rotation matrix of x and shoot
        //the vector through
        parallel_for(
            "Closest Point Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {d1.extent(0), d1.extent(1)}), KOKKOS_LAMBDA(int i, int j) {
                if (x(i, j).z != 0)
                {
                    Kvec3f px = Txy() * x(i, j);
                    Kvec2f c = Kvec2f(px.y * fy / px.z + ci, px.x * fx / px.z + cj);
                    int centeri = (int)c.x;
                    int centerj = (int)c.y;
                    //search for all near coordinates
                    float smallestDist = 1e6;
                    if (centeri > n_search && centeri < resi - n_search && centerj > n_search && centerj < resj - n_search)
                    {
                        for (int k = centeri - n_search; k < centeri + n_search; k++) //k for i-coordinate
                        {
                            for (int l = centerj - n_search; l < centerj + n_search; l++) //l for j-coordinate
                            {
                                Kvec3f py = y(k, l);
                                if (ICPType == PointToPoint)
                                {
                                    if (py.z != 0)
                                    {
                                        float dist = (py - px).length();
                                        if (dist < smallestDist && dist < max_dist)
                                        {
                                            smallestDist = dist;
                                            x_nearest(i, j) = py;
                                        }
                                    }
                                }
                                else if (ICPType == PointToPlane)
                                {
                                    if (py.z != 0 && y_normals(k, l).x != 0)
                                    {
                                        float d = (py - px) * y_normals(k, l);
                                        float dist = abs(d);
                                        if (dist < smallestDist && dist < max_dist)
                                        {
                                            smallestDist = dist;
                                            x_nearest(i, j) = px + d * y_normals(k, l);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            });


        //calculate center of mass for both pcds and rsme and fitness
        auto x_nearestMirror = create_mirror_view(x_nearest);
        deep_copy(x_nearestMirror, x_nearest);
        Eigen::Vector3d centerx = Eigen::Vector3d::Zero();
        Eigen::Vector3d centery = Eigen::Vector3d::Zero();

        float rsme = 0;
        float fitness = 0;
        float n_cors = 0;
        float n_pointsinX =0;
        for (int i = 0; i < x_nearest.extent(0); i++)
        {
            for (int j = 0; j < x_nearest.extent(1); j++)
            {
                if (x_nearestMirror(i, j).z != 0)
                {
                    centerx += Eigen::Vector3d(x_mirror(i, j).x, x_mirror(i, j).y, x_mirror(i, j).z);
                    centery += Eigen::Vector3d(x_nearestMirror(i, j).x, x_nearestMirror(i, j).y, x_nearestMirror(i, j).z);
                    n_cors++;
                    float tmp = (x_nearestMirror(i, j).toEigen4() - Txy_eigen * x_mirror(i, j).toEigen4()).norm();
                    rsme += tmp * tmp;
                }
                if (x_mirror(i,j).z!=0){n_pointsinX++;}
            }
        }

        centerx = centerx / n_cors;
        centery = centery / n_cors;
        rsme /= n_cors;
        rsme = sqrtf(rsme);
        fitness = n_cors / n_pointsinX;
        if (fitness > bestFitness)
        {
            bestFitness = fitness;
        }
        // visualizeNearestPoints(x_nearest, transformPCD(space, x, Txy), y);

        //Build H matrix for SVD extraction
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        int nCors = 0;
        for (int i = 0; i < x_mirror.extent(0); i++)
        {
            for (int j = 0; j < x_mirror.extent(1); j++)
            {
                if (x_mirror(i, j).z != 0 && x_nearestMirror(i, j).z != 0)
                {
                    Eigen::Vector3d xn = Eigen::Vector3d(x_mirror(i, j).x, x_mirror(i, j).y, x_mirror(i, j).z);
                    Eigen::Vector3d yn = Eigen::Vector3d(x_nearestMirror(i, j).x, x_nearestMirror(i, j).y, x_nearestMirror(i, j).z);
                    Eigen::Matrix3d summand = (xn - centerx) * (yn - centery).transpose();
                    H = H + summand;
                    nCors++;
                }
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd;
        svd.compute(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixV() * svd.matrixU().transpose();
        Txy_eigen = Eigen::Matrix4d::Identity();
        Txy_eigen.block<3, 3>(0, 0) = R;
        Txy_eigen.block<3, 1>(0, 3) = centery - R * centerx;
        Txy = EigentoKmat4f(Txy_eigen.cast<float>());

        if (rsme < lowestRSME)
        {
            lowestRSME = rsme;
            bestTxy_eigen = Txy_eigen;
        }
        //reset x_nearest. don't for last iteration to calculate rsme
        if (iteration != n_it - 1)
        {
            parallel_for(
                "X_nearest Zero Kernel", MDRangePolicy<Cuda, Rank<2>>(space, {0, 0}, {x_nearest.extent(0), x_nearest.extent(1)}), KOKKOS_LAMBDA(int i, int j) {
                    x_nearest(i, j) = Kvec3f(0.0, 0.0, 0.0);
                });
        }
    }
    if (rsme_extern != nullptr)
    {
        *rsme_extern = lowestRSME;
    }
    cout << " with rsme " << lowestRSME << " and fitness " << bestFitness << endl;
    return bestTxy_eigen;

    //strategy: Align x to y with icp.
    //1. Use projective association on gpu to find closest points
    //2. Still on gpu use parallel_reduce to compute the H matrix (maybe probs with parallel_reduce)
    //3. on cpu perform svd
    //4. transform shit on gpu
    //5. Repeat

    // return Eigen::Matrix4d::Zero();
}
template Eigen::Matrix4d kokkosICP<Kokkos::Cuda>(Kokkos::Cuda &space, Kokkos::View<unsigned short **> p1, Kokkos::View<unsigned short **> p2,
                                                 const Eigen::Matrix4d &t1, const Eigen::Matrix4d &t2, ICPTypes ICPType, float *rsme_extern);
