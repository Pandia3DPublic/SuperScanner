#pragma once
#include "kokkosFunctions.h"
#include "core/Frame.h"

//this is stupid but necessary for fixed array length
constexpr int voxelPerEdge = 8;
constexpr int voxelPerEdgeSquared = 64;
constexpr int voxelPerBlock = 512; //alway pow(voxelPerEdge,3)
// constexpr int maxVBIndices = 1e5;

struct Voxel
{
    float sdf = 0;
    short w = 0; //must be short since cap doesnt work with deintegrate
    float r = 0;
    float g = 0;
    float b = 0;
};

struct Voxelblock
{
    Voxel voxel[voxelPerBlock] = {};
    int alreadyIntegrated = 0; //int to make atomic compare fast, bool is super slow
    Kvec3f pzero;              //zero point
    Kvec3i vbInd;              //voxel Block index

    KOKKOS_INLINE_FUNCTION
    Voxel *getVoxel(Kvec3i a)
    {
        return &voxel[a.x + a.y * voxelPerEdge + a.z * voxelPerEdgeSquared];
    }

    KOKKOS_INLINE_FUNCTION
    Voxel *getVoxel(int a)
    {
        return &voxel[a];
    }
};

class GPUVoxelGrid
{
public:
    GPUVoxelGrid(unsigned int c, float trunc, float voxlength, int resi_, int resj_);

    //variables
    bool intrinsicSet = false;
    float d_trunc;
    float vl;  //voxel length
    float vbl; //voxel block length
    int resi;  //todo check if these should follow from the intrinsic
    int resj;
    open3d::camera::PinholeCameraIntrinsic intr;
    int nVoxelEdge = voxelPerEdge;  //number of Voxel per Voxel Block Edge
    int nVoxelCube = voxelPerBlock; //number of Voxel per Voxel Block Edge
    float d_max = 10.0;             //max distance for raycasting //todo read from config for maxdepth
    float rmin = 0.2;               //minimal depth for a point to be integrated
    //Note: Try to never adress more than a cubic volume of 500x500x500 otherwise due to some weird 32 bit stuff
    // we will get stuff at completely the wrong place. This is not yet understood!
    Kokkos::UnorderedMap<unsigned long long, Voxelblock> voxelMap2;
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    int maxVoxelBlocks; //maximum number of voxel blocks possible in gpu mem with constructor given capacity
    int wmax = 30; //maximum weight in one voxel
    //Kokkos::UnorderedMap<Kvec3i,Voxel,Kokkos::DefaultExecutionSpace, PrimeHasher> voxelMap;

    //views
    Kokkos::View<Kvec3f **> inputPcd;          //init to resi/resj in constructor
    Kokkos::View<Kvec3f **> rayPcd;            //for rayCasting results
    // Kokkos::View<Kvec3f **> rayNormals;        //for rayCasting results
    Kokkos::View<Kvec3f **> rayColorPcd;       //for rayCasting color results
    Kokkos::View<Kvec3f **> normPcd;           //to speed up raycasting
    Kokkos::View<unsigned char **[3]> rgb;     //we shallow copy into these for kernel dispatch
    Kokkos::View<unsigned short **> depth;     //we shallow copy into these for kernel dispatch
    Kokkos::View<float[4][4]> E;               //The extrinisc matrix
    Kokkos::View<float[4][4]> E_inv;           //The inverse extrinisc matrix
    Kokkos::View<float[4][4]> E_ray;           //The extrinisc matrix for raycasting
    Kokkos::View<int*> vbIndices; 
    Kokkos::View<int> vbIndicesCount;          //This needs to be a view so that the actual data does not get copied

    //test & debug vars
    Kokkos::View<unsigned short **> testdepth;
    Kokkos::View<Kvec3f **> visPcd2;
    Kokkos::View<float **[3]> colorvisPcd2;
    bool all = false;

    //intrinsic
    float cx;
    float cy;
    float ci;
    float cj;
    float fx;
    float fy;

    //extrinsic
    Kvec3f Ez; //transformed z-axis;

    //non kokkos functions
    //note that frame passes in second integrate need frame which is only for debug toto
    template <typename SpaceType>
    void integrate(SpaceType &space, Kokkos::View<unsigned char **[3]> rgb_, Kokkos::View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor = true);
    template <typename SpaceType>
    void integrate(SpaceType &space, std::shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor = true);
    template <typename SpaceType>
    void deIntegrate(SpaceType &space,Kokkos::View<unsigned char **[3]> rgb_, Kokkos::View<unsigned short **> depth_, const Eigen::Matrix4f &extr, bool withColor = true);
    template <typename SpaceType>
    void deIntegrate(SpaceType &space, std::shared_ptr<Frame> f, const Eigen::Matrix4f &extr, bool withColor = true);

    template <typename SpaceType>
    void raycast(SpaceType &space,const Eigen::Matrix4f &extr);
    //void raycast(const Eigen::Matrix4f& extr, Kokkos::View<unsigned short**> depth_);
    template <typename SpaceType>
    void visualizeTSDF(SpaceType &space, bool all = false); //only allocates memory if called
    template <typename SpaceType>
    void testfunc(SpaceType &space); //Kokkos::View<unsigned short**> depth_);
    void eigentoView(Kokkos::View<float[4][4]> V, const Eigen::Matrix4f &M_eig);
    void ViewtoEigen(Eigen::Matrix4f &M_eig, Kokkos::View<float[4][4]> V);
    template <typename SpaceType>
    void cleanMemory(SpaceType &space);
    template <typename SpaceType>
    void setIntrinsic(SpaceType &space, const open3d::camera::PinholeCameraIntrinsic &intr_);
    void marchingCubesHost();

    //kokkos functions

    KOKKOS_INLINE_FUNCTION
    unsigned long long getPrimeHash(Kvec3i a) const
    {
        return 73856093 * a.x + 19349669 * a.y + 83492791 * a.z;
    }

    //get voxel indeces assuming dense structure
    KOKKOS_INLINE_FUNCTION Kvec3i getIndeces(Kvec3f a) const
    {
        Kvec3i out;
        if (a.x >= 0)
        {
            out.x = a.x / vl;
        }
        else
        {
            out.x = a.x / vl - 1;
        }
        if (a.y >= 0)
        {
            out.y = a.y / vl;
        }
        else
        {
            out.y = a.y / vl - 1;
        }
        if (a.z >= 0)
        {
            out.z = a.z / vl;
        }
        else
        {
            out.z = a.z / vl - 1;
        }
        return out;
    }

    //get voxel block indeces
    KOKKOS_INLINE_FUNCTION Kvec3i getVBIndeces(Kvec3f a) const
    {
        Kvec3i out;
        if (a.x >= 0)
        {
            out.x = a.x / vbl;
        }
        else
        {
            out.x = a.x / vbl - 1;
        }
        if (a.y >= 0)
        {
            out.y = a.y / vbl;
        }
        else
        {
            out.y = a.y / vbl - 1;
        }
        if (a.z >= 0)
        {
            out.z = a.z / vbl;
        }
        else
        {
            out.z = a.z / vbl - 1;
        }
        return out;
    }

    KOKKOS_INLINE_FUNCTION Kvec3i getVinVBIndeces(Kvec3f p, Kvec3i vbInd) const
    {
        p = p - vbl * vbInd;
        Kvec3i out;
        if (p.x >= 0)
        {
            out.x = p.x / vl;
        }
        else
        {
            out.x = p.x / vl - 1;
        }
        if (p.y >= 0)
        {
            out.y = p.y / vl;
        }
        else
        {
            out.y = p.y / vl - 1;
        }
        if (p.z >= 0)
        {
            out.z = p.z / vl;
        }
        else
        {
            out.z = p.z / vl - 1;
        }
        return out;
    }

    KOKKOS_INLINE_FUNCTION
    unsigned long long vbPointToHash(Kvec3f a) const
    {
        return getPrimeHash(getVBIndeces(a));
    }

    KOKKOS_INLINE_FUNCTION Kvec3f getPoint(int i, int j, float d) const
    {
        return Kvec3f((j - cj) / fx * d, (i - ci) / fy * d, d);
    }

    KOKKOS_INLINE_FUNCTION Kvec2f getCoordinate(Kvec3f p) const
    {
        return Kvec2f(p.y * fy / p.z + ci, p.x * fx / p.z + cj);
    }

    //todo maybe simd
    KOKKOS_INLINE_FUNCTION Kvec3f transformPoint(Kvec3f p, Kokkos::View<float[4][4]> E_) const
    {
        Kvec3f out;
        out.x = p.x * E_(0, 0) + p.y * E_(0, 1) + p.z * E_(0, 2) + E_(0, 3);
        out.y = p.x * E_(1, 0) + p.y * E_(1, 1) + p.z * E_(1, 2) + E_(1, 3);
        out.z = p.x * E_(2, 0) + p.y * E_(2, 1) + p.z * E_(2, 2) + E_(2, 3);
        return out;
    }

    KOKKOS_INLINE_FUNCTION bool matrixEqual(Kokkos::View<float[4][4]> A, Kokkos::View<float[4][4]> B) const
    {
        for (int i = 0; i < A.extent(0); i++)
        {
            for (int j = 0; j < A.extent(1); j++)
            {
                if (A(i, j) != B(i, j))
                    return false;
            }
        }
        return true;
    }

    KOKKOS_INLINE_FUNCTION bool kokkosIsPositive(float a) const
    {
        if (a >= 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    KOKKOS_INLINE_FUNCTION
    Kvec3f getVoxelMidPoint(Kvec3i v_inds, Kvec3i vb_inds) const
    {
        return Kvec3f(vb_inds.x * vbl + (v_inds.x + 0.5) * vl, vb_inds.y * vbl + (v_inds.y + 0.5) * vl, vb_inds.z * vbl + (v_inds.z + 0.5) * vl);
    }

    KOKKOS_FUNCTION void updateSingleVoxel(Voxel &v, const Kvec3f vm) const;
    KOKKOS_FUNCTION void deUpdateSingleVoxel(Voxel &v, const Kvec3f vm) const;
    KOKKOS_FUNCTION void updateSingleVoxelColor(Voxel &v, const Kvec3f vm) const;
    KOKKOS_FUNCTION void deUpdateSingleVoxelColor(Voxel &v, const Kvec3f vm) const;

    //template functions
    template <typename T>
    bool checkdims(T myview)
    {
        if (myview.extent(0) != resi || myview.extent(1) != resj)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    //operators
    class depthToPcdTag
    {
    };
    KOKKOS_INLINE_FUNCTION
    void operator()(depthToPcdTag, int i, int j) const
    {
        inputPcd(i, j) = getPoint(i, j, (float)(depth(i, j)) / 1000); //note: works faster with explicit typecast
    }

    class zeroVBIndecesTag
    {
    };
    KOKKOS_INLINE_FUNCTION
    void operator()(zeroVBIndecesTag, int i) const
    {
        vbIndices(i) = 0;
    }

    class zeroVBIndecesCounterTag
    {
    };
    KOKKOS_INLINE_FUNCTION
    void operator()(zeroVBIndecesCounterTag, int i) const
    {
        vbIndicesCount() = 0;
    }

    KOKKOS_INLINE_FUNCTION
    float getSDF(Kvec3f p)
    {
        Kvec3i vbInd = getVBIndeces(p);
        return voxelMap2.value_at(voxelMap2.find(getPrimeHash(vbInd))).getVoxel(getVinVBIndeces(p, vbInd))->sdf;
    }

    class vbCollectionTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(vbCollectionTag, int i, int j) const;


    class integrationCoreTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(integrationCoreTag, int i, int k, int l, int m) const;

    class integrationCoreColorTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(integrationCoreColorTag, int i, int k, int l, int m) const;

    class deIntegrationCoreTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(deIntegrationCoreTag, int i, int k, int l, int m) const;

    class deIntegrationCoreColorTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(deIntegrationCoreColorTag, int i, int k, int l, int m) const;


    class resetTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(resetTag, int i) const;


    class raycastingTag
    {
    };
    //raycast stuff just from the positive side.
    //note: this is simple raycasting without trilinear interpolation.
    KOKKOS_FUNCTION
    void operator()(raycastingTag, int i, int j) const;

    class createNormPcdTag
    {
    };
    //todo later: change this to take the intrinsic parameters of the virtual camera.
    KOKKOS_FUNCTION
    void operator()(createNormPcdTag, int i, int j) const;

    class visualizeTag2
    {
    };
    KOKKOS_FUNCTION
    void operator()(visualizeTag2, int i, int j, int k) const;

    class cleanTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(cleanTag, int i) const;

    //this performs normal calculation in the ray pcd
    class raycastingNormalsTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(raycastingNormalsTag, int i, int j) const;

    //faster too.
    class testTag
    {
    };
    KOKKOS_FUNCTION
    void operator()(testTag, int i, int j) const;

    class testTag2
    {
    };
    KOKKOS_FUNCTION
    void operator()(testTag2, int i, int j) const;
    
};