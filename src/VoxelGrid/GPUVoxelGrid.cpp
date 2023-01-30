#include "GPUVoxelGrid.h"
#include "utils/visutil.h"
#include "utils/imageutil.h"
#include "core/threadCom.h"
// #define DEBUGVOXELGRID

using namespace std;
using namespace Kokkos;

GPUVoxelGrid::GPUVoxelGrid(unsigned int c, float trunc,
                           float voxlength, int resi_, int resj_) : voxelMap2{Kokkos::UnorderedMap<unsigned long long, Voxelblock>(c / voxelPerBlock)},
                                                                    d_trunc{trunc}, vl{voxlength}, resi{resi_}, resj{resj_}
{

    vbl = nVoxelEdge * vl;

    inputPcd = View<Kvec3f **>("inputPcd", resi, resj);
    rayPcd = View<Kvec3f **>("rayPcd", resi, resj);
    // rayNormals = View<Kvec3f **>("rayNormals", resi, resj);
    rayColorPcd = View<Kvec3f **>("rayColorPcd", resi, resj);
    normPcd = View<Kvec3f **>("normPcd", resi, resj);
    E = Kokkos::View<float[4][4]>("Extrinsic matrix");
    E_inv = Kokkos::View<float[4][4]>("Inverse Extrinsic matrix");
    E_ray = Kokkos::View<float[4][4]>("Raycasting extrinsic matrix");
    mesh = make_shared<open3d::geometry::TriangleMesh>();

    maxVoxelBlocks = c / voxelPerBlock;
    vbIndices = Kokkos::View<int *>("VB Indices", maxVoxelBlocks);
    vbIndicesCount = Kokkos::View<int>("VB Indices Count");
}

template <typename SpaceType>
void GPUVoxelGrid::integrate(SpaceType &space, shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor)
{
    // f->integratecounter++;
    f->MovetoGPU();
    integrate(space, f->rgb, f->depth, extr, withColor);
}
template void GPUVoxelGrid::integrate<Kokkos::Cuda>(Kokkos::Cuda &space, shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor);

template <typename SpaceType>
void GPUVoxelGrid::integrate(SpaceType &space, View<unsigned char **[3]> rgb_, View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor)
{
    rgb = rgb_;     // shallow copies
    depth = depth_; // shallow copies
    if (!checkdims(rgb) || !checkdims(depth))
    {
        cout << "The input picture dimensions are not correct, they need to match the constructor dimensions!" << endl;
    }

    // Eigen to kokkos stuff
    Ez = Kvec3f(extr(0, 2), extr(1, 2), extr(2, 2)); // third column of extrinsic matrix;
    Ez.normalize();

    Eigen::Matrix4f extr_inv = extr.inverse();
    eigentoView(E, extr);
    eigentoView(E_inv, extr_inv);

    parallel_for("Depth_to_pcd", MDRangePolicy<depthToPcdTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);

    parallel_for("Integration Collection Kernel", MDRangePolicy<vbCollectionTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
    auto count_h = create_mirror_view(vbIndicesCount);
    deep_copy(count_h, vbIndicesCount);
    // cout << "count is " << count_h() << endl;
    if (withColor)
    {
        parallel_for("Integration Core Kernel Color", MDRangePolicy<integrationCoreColorTag, Cuda, Rank<4>>(space, {0, 0, 0, 0}, {count_h(), nVoxelEdge, nVoxelEdge, nVoxelEdge}), *this);
    }
    else
    {
        parallel_for("Integration Core Kernel", MDRangePolicy<integrationCoreTag, Cuda, Rank<4>>(space, {0, 0, 0, 0}, {count_h(), nVoxelEdge, nVoxelEdge, nVoxelEdge}), *this);
    }

    // todo how slow are these simple kernel calls due to data copying? try deep_copy
    // kernel is stupid but 0.12ms seems okay. note, scales linear with size
    parallel_for("Reset booleans", RangePolicy<resetTag>(space, 0, voxelMap2.capacity()), *this);
    parallel_for("Reset Voxel Block Indeces", RangePolicy<zeroVBIndecesTag>(space, 0, maxVoxelBlocks), *this);
    parallel_for("Reset VB Indeces Counter", RangePolicy<zeroVBIndecesCounterTag>(space, 0, 1), *this);
    Kokkos::fence();

    // cout << intr.intrinsic_matrix_ << endl;
    threadCom::unmeshed_data = true;
}
template void GPUVoxelGrid::integrate<Kokkos::Cuda>(Kokkos::Cuda &space, View<unsigned char **[3]> rgb_, View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor);

template <typename SpaceType>
void GPUVoxelGrid::deIntegrate(SpaceType &space, shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor)
{

    f->MovetoGPU();
    deIntegrate(space, f->rgb, f->depth, extr, withColor);
}
template void GPUVoxelGrid::deIntegrate<Kokkos::Cuda>(Kokkos::Cuda &space, shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor);

template <typename SpaceType>
void GPUVoxelGrid::deIntegrate(SpaceType &space, View<unsigned char **[3]> rgb_, View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor)
{
    rgb = rgb_;     // shallow copies
    depth = depth_; // shallow copies
    if (!checkdims(rgb) || !checkdims(depth))
    {
        cout << "The input picture dimensions are not correct, they need to match the constructor dimensions!" << endl;
    }

    // Eigen to kokkos stuff
    Ez = Kvec3f(extr(0, 2), extr(1, 2), extr(2, 2)); // third column of extrinsic matrix;
    Ez.normalize();

    Eigen::Matrix4f extr_inv = extr.inverse();
    eigentoView(E, extr);
    eigentoView(E_inv, extr_inv);

    parallel_for("Depth_to_pcd", MDRangePolicy<depthToPcdTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);

    parallel_for("Deintegration Collection Kernel", MDRangePolicy<vbCollectionTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
    auto count_h = create_mirror_view(vbIndicesCount);
    deep_copy(count_h, vbIndicesCount);
    // cout << "count is " << count_h() << endl;
    if (withColor)
    {
        parallel_for("Deintegration Core Kernel Color", MDRangePolicy<deIntegrationCoreColorTag, Cuda, Rank<4>>(space, {0, 0, 0, 0}, {count_h(), nVoxelEdge, nVoxelEdge, nVoxelEdge}), *this);
    }
    else
    {
        parallel_for("Deintegration Core Kernel", MDRangePolicy<deIntegrationCoreTag, Cuda, Rank<4>>(space, {0, 0, 0, 0}, {count_h(), nVoxelEdge, nVoxelEdge, nVoxelEdge}), *this);
    }

    // todo how slow are these simple kernel calls due to data copying? try deep_copy
    // kernel is stupid but 0.12ms seems okay. note, scales linear with size
    parallel_for("Reset booleans", RangePolicy<resetTag>(space, 0, voxelMap2.capacity()), *this);
    parallel_for("Reset Voxel Block Indeces", RangePolicy<zeroVBIndecesTag>(space, 0, maxVoxelBlocks), *this);
    parallel_for("Reset VB Indeces Counter", RangePolicy<zeroVBIndecesCounterTag>(space, 0, 1), *this);
    Kokkos::fence();

    threadCom::unmeshed_data = true;
}
template void GPUVoxelGrid::deIntegrate<Kokkos::Cuda>(Kokkos::Cuda &space, View<unsigned char **[3]> rgb_, View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor);

template <typename SpaceType>
void GPUVoxelGrid::raycast(SpaceType &space, const Eigen::Matrix4f &extr)
{
    eigentoView(E_ray, extr);
    parallel_for("Raycasting Kernel", MDRangePolicy<raycastingTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
    // parallel_for("Raycasting Normal Kernel", MDRangePolicy<raycastingNormalsTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);

    Kokkos::fence();
}
template void GPUVoxelGrid::raycast<Kokkos::Cuda>(Kokkos::Cuda &space, const Eigen::Matrix4f &extr);

// this version supports speed up by using live depth information
// the depth image must be the image captured at the intrinsic
//  void GPUVoxelGrid::raycast(const Eigen::Matrix4f& extr,View<unsigned short**> depth_){
//      depth = depth_; // shallow copies
//      eigentoView(E_ray,extr);
//      parallel_for("Raycasting Cheat Kernel",MDRangePolicy<raycastingCheatTag,Cuda,Rank<2>>({0,0},{resi, resj}),*this);
//      // double time = timer.seconds();
//      //cout << "Time in seconds " << time << endl;
//  }

void GPUVoxelGrid::eigentoView(Kokkos::View<float[4][4]> V, const Eigen::Matrix4f &M_eig)
{
    auto V_h = create_mirror_view(V);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            V_h(i, j) = M_eig(i, j);
        }
    }
    deep_copy(V, V_h);
}

void GPUVoxelGrid::ViewtoEigen(Eigen::Matrix4f &M_eig, Kokkos::View<float[4][4]> V)
{
    auto V_h = create_mirror_view(V);
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            M_eig(i, j) = V_h(i, j);
        }
    }
}

template <typename SpaceType>
void GPUVoxelGrid::cleanMemory(SpaceType &space)
{
    // iterate over all blocks
    // for each block, check if all voxel weights are zero. maybe with parallel reduce
    // if no delete the entry using erase
    Kokkos::fence();
    voxelMap2.begin_erase();
    parallel_for("Collect non zero Blocks", RangePolicy<cleanTag>(space, 0, voxelMap2.capacity()), *this);
    voxelMap2.end_erase();
    Kokkos::fence();
}
template void GPUVoxelGrid::cleanMemory<Kokkos::Cuda>(Kokkos::Cuda &space);

// perform marching cubes by building an open3d tsdf volume on host
void GPUVoxelGrid::marchingCubesHost()
{
    using namespace open3d::pipelines;

    // deep copy voxelMap2 to host
    Kokkos::UnorderedMap<unsigned long long, Voxelblock, Kokkos::HostSpace> voxelMap_h;
    deep_copy(voxelMap_h, voxelMap2);
    Kokkos::fence();

    int capacity = voxelMap_h.capacity();
    int size = voxelMap_h.size();

    shared_ptr<integration::ScalableTSDFVolume> tsdf = make_shared<integration::ScalableTSDFVolume>(vl, d_trunc, integration::TSDFVolumeColorType::RGB8, nVoxelEdge);
    tsdf->volume_units_.reserve(size);

    for (int i = 0; i < capacity; i++)
    {
        if (voxelMap_h.valid_at(i))
        {
            Voxelblock &vb = voxelMap_h.value_at(i);
            Eigen::Vector3i index(vb.vbInd.x, vb.vbInd.y, vb.vbInd.z);

            integration::ScalableTSDFVolume::VolumeUnit unit; // o3d voxelblock
            unit.volume_.reset(new integration::UniformTSDFVolume(vl * nVoxelEdge, nVoxelEdge, d_trunc,
                                                                  integration::TSDFVolumeColorType::RGB8, index.cast<double>() * vl * nVoxelEdge));
            unit.index_ = index;

            int vcount = 0;
            for (int x = 0; x < nVoxelEdge; x++)
            {
                for (int y = 0; y < nVoxelEdge; y++)
                {
                    for (int z = 0; z < nVoxelEdge; z++)
                    {
                        int idx = x + y * nVoxelEdge + z * nVoxelEdge * nVoxelEdge;
                        Voxel &v = vb.voxel[vcount];
                        unit.volume_->voxels_[idx].color_ = Eigen::Vector3d(v.r, v.g, v.b);
                        unit.volume_->voxels_[idx].tsdf_ = v.sdf;
                        unit.volume_->voxels_[idx].weight_ = v.w;
                        vcount++;
                    }
                }
            }
            tsdf->volume_units_[index] = unit;
        }
    }

    *mesh = *tsdf->ExtractTriangleMesh();
    mesh->ComputeVertexNormals();

    threadCom::unmeshed_data = false;
}

template <typename SpaceType>
void GPUVoxelGrid::setIntrinsic(SpaceType &space, const open3d::camera::PinholeCameraIntrinsic &intr_)
{
    intr = intr_;
    cx = intr_.intrinsic_matrix_(0, 2);
    cy = intr_.intrinsic_matrix_(1, 2);
    ci = resi - cy;
    cj = cx;
    fx = intr_.intrinsic_matrix_(0, 0);
    fy = intr_.intrinsic_matrix_(1, 1);

    parallel_for("NormPcdKernel", MDRangePolicy<createNormPcdTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
    intrinsicSet = true;
}
template void GPUVoxelGrid::setIntrinsic<Kokkos::Cuda>(Kokkos::Cuda &space, const open3d::camera::PinholeCameraIntrinsic &intr_);

template <typename SpaceType>
void GPUVoxelGrid::visualizeTSDF(SpaceType &space, bool all)
{
    this->all = all;
    visPcd2 = View<Kvec3f **>("VisPcd2", voxelMap2.capacity(), nVoxelCube);
    colorvisPcd2 = View<float **[3]>("ColorVisPcd2", voxelMap2.capacity(), nVoxelCube);

    int vislimit = 500 / nVoxelEdge;
    parallel_for("Visualization Kernel", MDRangePolicy<visualizeTag2, Cuda, Rank<3>, IndexType<int>>(space, {-vislimit, -vislimit, -vislimit}, {vislimit, vislimit, vislimit}), *this);
    Kokkos::fence();

    auto visPcd_h = create_mirror_view(visPcd2);
    deep_copy(visPcd_h, visPcd2); // visPcd_h layout left
    auto colorvisPcd_h = create_mirror_view(colorvisPcd2);
    deep_copy(colorvisPcd_h, colorvisPcd2); // visPcd_h layout left

    // vis
    shared_ptr<open3d::geometry::PointCloud> o3pcd = make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < visPcd_h.extent(0); i++)
    {
        for (int j = 0; j < visPcd_h.extent(1); j++)
        {
            if (visPcd_h(i, j).x != 0 || visPcd_h(i, j).y != 0 || visPcd_h(i, j).z != 0)
            {
                Eigen::Vector3d pcd_heigen = Eigen::Vector3d(visPcd_h(i, j).x, visPcd_h(i, j).y, visPcd_h(i, j).z);
                Eigen::Vector3d color = Eigen::Vector3d(colorvisPcd_h(i, j, 0), colorvisPcd_h(i, j, 1), colorvisPcd_h(i, j, 2));
                o3pcd->points_.push_back(pcd_heigen);
                o3pcd->colors_.push_back(color);
            }
        }
    }

    auto pcd_h = create_mirror_view(rayPcd);
    deep_copy(pcd_h, rayPcd); // pcd_h layout left
    shared_ptr<open3d::geometry::PointCloud> ray03dpcd = make_shared<open3d::geometry::PointCloud>();

    for (int i = 0; i < resi; i++)
    {
        for (int j = 0; j < resj; j++)
        {
            // cout << pcd_h(j,i,2) << endl;
            Eigen::Vector3d pcd_heigen = Eigen::Vector3d(pcd_h(i, j).x, pcd_h(i, j).y, pcd_h(i, j).z);
            ray03dpcd->points_.push_back(pcd_heigen);
            // cout << pcd_heigen << endl;
        }
    }

    open3d::visualization::DrawGeometries({o3pcd, getOrigin()});
    // auto background = make_shared<open3d::geometry::Image>();
    // background->Prepare(640,480,3,1);
    // for (int i=0 ; i< background->width_; i++){
    //     for (int j=0 ; j< background->height_; j++){
    //         *(background->PointerAt<u_int8_t>(i,j,2)) = 255;
    //         *(background->PointerAt<u_int8_t>(i,j,1)) = 255;
    //         *(background->PointerAt<u_int8_t>(i,j,0)) = 255;
    //     }
    // }
    // open3d::visualization::DrawGeometries({o3pcd,getOrigin()});
}
template void GPUVoxelGrid::visualizeTSDF<Kokkos::Cuda>(Kokkos::Cuda &space, bool all);

template <typename SpaceType>
void GPUVoxelGrid::testfunc(SpaceType &space)
{ // Kokkos::View<unsigned short**> depth_){
    parallel_for("test2", MDRangePolicy<testTag2, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
    parallel_for("test", MDRangePolicy<testTag, Cuda, Rank<2>>(space, {0, 0}, {resi, resj}), *this);
}
template void GPUVoxelGrid::testfunc<Kokkos::Cuda>(Kokkos::Cuda &space);

// ############################# kernel functions ##############################
//  KOKKOS_INLINE_FUNCTION
//  unsigned long long GPUVoxelGrid::getPrimeHash(Kvec3i a) const
//  {
//      return 73856093 * a.x + 19349669 * a.y + 83492791 * a.z;
//  }

KOKKOS_FUNCTION void GPUVoxelGrid::updateSingleVoxel(Voxel &v, const Kvec3f vm) const
{
    Kvec2f coord = getCoordinate(transformPoint(vm, E_inv));
    int cori = (int)coord.x; // explicit cast is a tiny bit slower
    int corj = (int)coord.y;
    if (cori < resi && corj < resj && cori >= 0 && corj >= 0)
    {
        // get point from pixel
        Kvec3f vpt = inputPcd(cori, corj); // using a reference here is much slower! Not using this is also much slower. Must be a memory acces thing
        if (vpt.z > rmin)
        {
            vpt = transformPoint(vpt, E);
            float sdf = (vpt - vm) * Ez; // see derivation from Tim
            if (abs(sdf) < d_trunc)
            {
                v.sdf = (v.sdf * v.w + sdf) / (v.w + 1);
                v.w += 1;
            }
        }
    }
}

KOKKOS_FUNCTION void GPUVoxelGrid::deUpdateSingleVoxel(Voxel &v, const Kvec3f vm) const
{
    Kvec2f coord = getCoordinate(transformPoint(vm, E_inv));
    int cori = (int)coord.x; // explicit cast is a tiny bit slower
    int corj = (int)coord.y;
    if (cori < resi && corj < resj && cori >= 0 && corj >= 0)
    {
        // get point from pixel
        Kvec3f vpt = inputPcd(cori, corj); // using a reference here is much slower! Not using this is also much slower. Must be a memory acces thing
        if (vpt.z > rmin)
        {
            vpt = transformPoint(vpt, E);
            float sdf = (vpt - vm) * Ez; // see derivation from Tim
            if (abs(sdf) < d_trunc)
            {
                if (v.w != 1)
                {
                    v.sdf = (v.sdf * v.w - sdf) / (v.w - 1);
                    v.w -= 1; // todo only deintegrate if true weight is below wmax
                }
                else
                {
                    v.w = 0;
                    v.sdf = 0;
                }
            }
        }
    }
}

KOKKOS_FUNCTION void GPUVoxelGrid::updateSingleVoxelColor(Voxel &v, const Kvec3f vm) const
{
    Kvec2f coord = getCoordinate(transformPoint(vm, E_inv));
    int cori = (int)coord.x; // explicit cast is a tiny bit slower
    int corj = (int)coord.y;
    if (cori < resi && corj < resj && cori >= 0 && corj >= 0)
    {
        // get point from pixel
        Kvec3f vpt = inputPcd(cori, corj); // using a reference here is much slower! Not using this is also much slower. Must be a memory acces thing
        if (vpt.z > rmin)
        {
            vpt = transformPoint(vpt, E);
            float sdf = (vpt - vm) * Ez; // see derivation from Tim
            if (abs(sdf) < d_trunc)
            {
                v.sdf = (v.sdf * v.w + sdf) / (v.w + 1);
                // if (v.w == 0){
                v.r = (v.r * v.w + rgb(cori, corj, 0)) / (v.w + 1);
                v.g = (v.g * v.w + rgb(cori, corj, 1)) / (v.w + 1);
                v.b = (v.b * v.w + rgb(cori, corj, 2)) / (v.w + 1);
                // }
                // v.color = Kvec3b(rgb(cori, corj, 0), rgb(cori, corj, 1), rgb(cori, corj, 2)); // for simple check
                // printf("color %i %i %i \n", v.color.x, v.color.y, v.color.z);
                v.w += 1;
            }
        }
    }
}

KOKKOS_FUNCTION void GPUVoxelGrid::deUpdateSingleVoxelColor(Voxel &v, const Kvec3f vm) const
{
    Kvec2f coord = getCoordinate(transformPoint(vm, E_inv));
    int cori = (int)coord.x; // explicit cast is a tiny bit slower
    int corj = (int)coord.y;
    if (cori < resi && corj < resj && cori >= 0 && corj >= 0)
    {
        // get point from pixel
        Kvec3f vpt = inputPcd(cori, corj); // using a reference here is much slower! Not using this is also much slower. Must be a memory acces thing
        if (vpt.z > rmin)
        {
            vpt = transformPoint(vpt, E);
            float sdf = (vpt - vm) * Ez; // see derivation from Tim
            if (abs(sdf) < d_trunc)
            {
                if (v.w != 1)
                {
#ifdef DEBUGVOXELGRID
                    if (v.w == 0)
                    {
                        printf("v.w is zero in deintegrate! \n");
                    }
#endif
                    // todo maybe creat a v.w -1 variable
                    v.sdf = (v.sdf * v.w - sdf) / (v.w - 1);
                    // if (v.w == 1){
                    v.r = (v.r * v.w - rgb(cori, corj, 0)) / (v.w - 1);
                    v.g = (v.g * v.w - rgb(cori, corj, 1)) / (v.w - 1);
                    v.b = (v.b * v.w - rgb(cori, corj, 2)) / (v.w - 1);
                    // }
                    v.w -= 1;
                }
                else
                {
                    v.w = 0;
                    v.sdf = 0;
                    v.r = 0;
                    v.g = 0;
                    v.b = 0;
                }
            }
        }
    }
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(vbCollectionTag, int i, int j) const
{
    // calcualte Voxel coordinates of point
    Kvec3f p = inputPcd(i, j);
    // printf("%f %f %f \n", p.x,p.y,p.z);
    if (p.z > rmin)
    {
        Kvec3f pt = transformPoint(p, E); // transformed point
        // calculate circumference of influence
        Kvec3i minIndeces = getVBIndeces(pt - Kvec3f(d_trunc, d_trunc, d_trunc));
        Kvec3i maxIndeces = getVBIndeces(pt + Kvec3f(d_trunc, d_trunc, d_trunc));
        unsigned int index;
        // for all affected voxel do
        for (int x = minIndeces.x; x <= maxIndeces.x; x++)
        {
            for (int y = minIndeces.y; y <= maxIndeces.y; y++)
            {
                for (int z = minIndeces.z; z <= maxIndeces.z; z++)
                {
                    // now we are inside one voxel block
                    // allocate voxel block if not already done
                    auto hash = getPrimeHash(Kvec3i(x, y, z));
                    index = voxelMap2.find(hash);
                    if (!voxelMap2.valid_at(index))
                    {
                        auto r = voxelMap2.insert(hash); // default constructed. insert here is madly expensive
                        index = r.index();
                    }
                    Voxelblock &vb = voxelMap2.value_at(index);
                    vb.pzero = vbl * Kvec3f(x, y, z);
                    vb.vbInd = Kvec3i(x, y, z);
                    // until here we average 0.65ms including the atomic
                    if (!Kokkos::atomic_exchange(&vb.alreadyIntegrated, 1))
                    {
                        int viewIndex = Kokkos::atomic_fetch_add(&vbIndicesCount(), 1); // returns old value
                        // printf("viewindex %i \n", viewIndex);
                        vbIndices(viewIndex) = index;
                    }
                }
            }
        }
    }
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(integrationCoreTag, int i, int k, int l, int m) const
{
    Voxelblock &vb = voxelMap2.value_at(vbIndices(i));
    Voxel *v = vb.getVoxel(Kvec3i(k, l, m));
    Kvec3f vm(vb.pzero.x + (k + 0.5) * vl, vb.pzero.y + (l + 0.5) * vl, vb.pzero.z + (m + 0.5) * vl);
    updateSingleVoxel(*v, vm);
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(integrationCoreColorTag, int i, int k, int l, int m) const
{
    Voxelblock &vb = voxelMap2.value_at(vbIndices(i));
    Voxel *v = vb.getVoxel(Kvec3i(k, l, m));
    Kvec3f vm(vb.pzero.x + (k + 0.5) * vl, vb.pzero.y + (l + 0.5) * vl, vb.pzero.z + (m + 0.5) * vl);
    updateSingleVoxelColor(*v, vm);
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(deIntegrationCoreTag, int i, int k, int l, int m) const
{
    Voxelblock &vb = voxelMap2.value_at(vbIndices(i));
    Voxel *v = vb.getVoxel(Kvec3i(k, l, m));
    Kvec3f vm(vb.pzero.x + (k + 0.5) * vl, vb.pzero.y + (l + 0.5) * vl, vb.pzero.z + (m + 0.5) * vl);
    deUpdateSingleVoxel(*v, vm);
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(deIntegrationCoreColorTag, int i, int k, int l, int m) const
{
    Voxelblock &vb = voxelMap2.value_at(vbIndices(i));
    Voxel *v = vb.getVoxel(Kvec3i(k, l, m));
    Kvec3f vm(vb.pzero.x + (k + 0.5) * vl, vb.pzero.y + (l + 0.5) * vl, vb.pzero.z + (m + 0.5) * vl);
    deUpdateSingleVoxelColor(*v, vm);
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(resetTag, int i) const
{
    if (voxelMap2.valid_at(i))
    {
        voxelMap2.value_at(i).alreadyIntegrated = false;
    }
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(raycastingTag, int i, int j) const
{
    // Kvec3f rv = transformPoint(normPcd(i,j),E_ray);
    Kvec3f rv(transformPoint(normPcd(i, j), E_ray));
    Kvec3f dir = rv - Kvec3f(E_ray(0, 3), E_ray(1, 3), E_ray(2, 3)); // is normalized here
    // step along the ray until a voxel exists
    float l = 0.1; // start at 10 cm mindist, distance travelled
    rv -= (1 - l) * dir;

    // for cheat
    //  float l;
    //  if (depth(i, j) != 0)
    //  {
    //      l = (float)(depth(i, j)) / 1000 - 0.1;
    //      rv += (l - 1) * dir; //minus 1 since rv is 1m long.
    //  }
    //  else
    //  {
    //      l = 0.1; //start at 10 cm mindist, distance travelled
    //      rv -= (1 - l) * dir;
    //  }

    bool done = false;
    bool Fn = false; // indicates that the first encountered voxel is negative
    bool Lp = false;
    Kvec3i oldvbInd;

    while (!done && !Fn && l < d_max)
    {
        Kvec3i vbInd = getVBIndeces(rv);
        auto hash = getPrimeHash(vbInd);
        unsigned int index = voxelMap2.find(hash);
        int count = 0;
        // go until you find the first vb
        while (!voxelMap2.valid_at(index) && l < d_max)
        {
            rv += vbl * dir;
            l += vbl;
            vbInd = getVBIndeces(rv);
            hash = getPrimeHash(vbInd);
            index = voxelMap2.find(hash);
            count++;
        }
        if (l >= d_max)
        {
            break;
        }
        // if we actually stepped with vbl, we need to go back a bit
        if (count > 0)
        {
            // go back one and forward d_trunc
            rv -= (vbl - d_trunc) * dir;
            l -= (vbl - d_trunc);
            // renew variables
            vbInd = getVBIndeces(rv);
            hash = getPrimeHash(vbInd);
            index = voxelMap2.find(hash);
            while (!voxelMap2.valid_at(index) && l < d_max) // dont need l check here
            {
                rv += d_trunc * dir;
                l += d_trunc;
                vbInd = getVBIndeces(rv);
                hash = getPrimeHash(vbInd);
                index = voxelMap2.find(hash);
            }
        }
        // ######## end of going back ##################
        if (l >= d_max)
        {
            break;
        }
        Voxelblock &vb = voxelMap2.value_at(index);
        Kvec3i vInd = getVinVBIndeces(rv, vbInd); // get the coordinates of the voxel in which rv lies within the block.
        oldvbInd = vbInd;

        // inner loop, advance within voxel block. vb must always be correct here
        while (!done && l < d_max && oldvbInd == vbInd && !Fn)
        {
            Voxel *cv = vb.getVoxel(vInd);
            if (cv->w > 0)
            {
                if (cv->sdf > 0)
                {
                    Lp = true;
                    // increase
                    rv += vl * dir;
                    l += vl;
                    oldvbInd = vbInd;
                    vbInd = getVBIndeces(rv);
                    vInd = getVinVBIndeces(rv, vbInd);
                }
                else
                {
                    if (Lp)
                    {
                        // create Point
                        done = true; // zero crossing detected
                        // Voxel v_in = cv;
                        Kvec3f vm_in = getVoxelMidPoint(vInd, vbInd);
                        // go one back
                        rv -= vl * dir;
                        l -= vl;
                        // calculate everything anew in case we entered a new voxelblock
                        vbInd = getVBIndeces(rv); // no need to update the old index here
                        vInd = getVinVBIndeces(rv, vbInd);
                        Kvec3f vm_out = getVoxelMidPoint(vInd, vbInd);
                        Voxel *v_out;
                        if (vbInd != oldvbInd)
                        {
                            Voxelblock &vb2 = voxelMap2.value_at(voxelMap2.find(getPrimeHash(vbInd)));
                            v_out = vb2.getVoxel(vInd);
                        }
                        else
                        {
                            v_out = vb.getVoxel(vInd);
                        }
                        float x = (v_out->w * v_out->sdf / vl + cv->w * (1 - cv->sdf / vl)) / (cv->w + v_out->w);
                        Kvec3f p = vm_out + x * (vm_in - vm_out);
                        // rayPcd(i,j) = p; //this is the variant where is point lies between two voxels
                        rayPcd(i, j) = rv + (p * dir - rv * dir) * dir;             // this is the variant where is point lies exactly on the ray
                        rayColorPcd(i, j).x = (0.5 * v_out->r + 0.5 * cv->r) / 255; // simple //todo only do this in case its needed
                        rayColorPcd(i, j).y = (0.5 * v_out->g + 0.5 * cv->g) / 255; // simple //todo only do this in case its needed
                        rayColorPcd(i, j).z = (0.5 * v_out->b + 0.5 * cv->b) / 255; // simple //todo only do this in case its needed
                        if (v_out->w == 0)
                        {
                            // printf("error \n");
                        }
                    }
                    else
                    {
                        Fn = true;
                    }
                }
            }
            else
            {
                Lp = false;
                // increase
                rv += vl * dir;
                l += vl;
                oldvbInd = vbInd;
                vbInd = getVBIndeces(rv);
                vInd = getVinVBIndeces(rv, vbInd);
            }
        }
    }
    if (!done)
    {
        rayPcd(i, j) = Kvec3f(0.0, 0.0, 0.0);
        rayColorPcd(i, j).x = 0; // todo only do this in case its needed
        rayColorPcd(i, j).y = 0; // todo only do this in case its needed
        rayColorPcd(i, j).z = 0; // todo only do this in case its needed
    }
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(createNormPcdTag, int i, int j) const
{
    normPcd(i, j) = getPoint(i, j, 1.0);
    normPcd(i, j).normalize();
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(visualizeTag2, int i, int j, int k) const
{
    auto hash = getPrimeHash(Kvec3i(i, j, k));
    auto index = voxelMap2.find(hash);
    if (voxelMap2.valid_at(index))
    {
        for (int l = 0; l < nVoxelEdge; l++)
        {
            for (int m = 0; m < nVoxelEdge; m++)
            {
                for (int n = 0; n < nVoxelEdge; n++)
                {
                    int innerIndex = l + m * nVoxelEdge + n * nVoxelEdge * nVoxelEdge;
                    if (voxelMap2.value_at(index).voxel[innerIndex].w != 0)
                    {
                        visPcd2(index, innerIndex) = Kvec3f(i * vbl + (l + 0.5) * vl, j * vbl + (m + 0.5) * vl, k * vbl + (n + 0.5) * vl);
                        float val = voxelMap2.value_at(index).voxel[innerIndex].sdf;
                        if (val > 0)
                        {
                            val = val / d_trunc;
                            if (val > 1)
                            {
                                val = 1;
                            }
                            colorvisPcd2(index, innerIndex, 0) = 1; // red is positive
                        }
                        else
                        {
                            val = abs(val);
                            val = val / d_trunc;
                            if (val > 1)
                            {
                                val = 1;
                            }
                            colorvisPcd2(index, innerIndex, 2) = 1; // blue is negative
                        }
                        if (val == 0)
                        {
                            colorvisPcd2(index, innerIndex, 0) = 0;
                            colorvisPcd2(index, innerIndex, 2) = 0;
                        }
                        colorvisPcd2(index, innerIndex, 1) = 0;
                        // to see only in trunc allocation comment out this else part
                    }
                    else
                    {
                        if (all)
                        {
                            visPcd2(index, innerIndex) = Kvec3f(i * vbl + (l + 0.5) * vl, j * vbl + (m + 0.5) * vl, k * vbl + (n + 0.5) * vl);
                            colorvisPcd2(index, innerIndex, 0) = 0.5;
                            colorvisPcd2(index, innerIndex, 1) = 0.5;
                            colorvisPcd2(index, innerIndex, 2) = 0.5;
                        }
                    }
                }
            }
        }
    }
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(cleanTag, int i) const
{
    if (voxelMap2.valid_at(i))
    {
        bool empty = true;
        // block is allocated
        for (int j = 0; j < nVoxelCube; j++)
        {
            if (voxelMap2.value_at(i).getVoxel(j)->w != 0)
            {
                empty = false;
            }
        }
        if (empty)
        {
            // block contains nothing
            bool a = voxelMap2.erase(voxelMap2.key_at(i));
        }
    }
}

// KOKKOS_FUNCTION
// void GPUVoxelGrid::operator()(raycastingNormalsTag, int i, int j) const
// {
//     Kvec3f v1, v2;
//     Kvec3f p = rayPcd(i, j);
//     if (i == resi - 1 && j != resj - 1)
//     { //end row
//         v1 = rayPcd(i, j + 1) - p;
//         v2 = rayPcd(i - 1, j) - p;
//     }
//     if (i != resi - 1 && j == resj - 1)
//     { //end column
//         v1 = rayPcd(i, j - 1) - p;
//         v2 = rayPcd(i + 1, j) - p;
//     }
//     if (i == resi - 1 && j == resj - 1)
//     { //end row and end column
//         v1 = rayPcd(i, j - 1) - p;
//         v2 = rayPcd(i - 1, j) - p;
//     }
//     else
//     {
//         v1 = rayPcd(i, j + 1) - p;
//         v2 = rayPcd(i + 1, j) - p;
//     }
//     Kvec3f tmp = v1.cross(v2);
//     tmp.normalize();
//     rayNormals(i, j) = tmp;
// }

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(testTag, int i, int j) const
{
    // Kvec3f a(i, i, i);
    // Kvec3f b(j, j, j);
    // Kvec3f c = a + b;
}

KOKKOS_FUNCTION
void GPUVoxelGrid::operator()(testTag2, int i, int j) const
{
    // float a1 = 5;
    // float a2 = 5;
    // float a3 = 5;
    // float b1 = 5;
    // float b2 = 5;
    // float b3 = 5;
    // float c1 = a1+b1;
    // float c2 = a2+b2;
    // float c3 = a3+b3;
}