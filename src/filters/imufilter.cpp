#include "imufilter.h"
#include "core/integrate.h"
#include "utils/coreutil.h"
#include "core/threadCom.h"
#include "utils/matrixutil.h"
using namespace std;
//this is the stupid variant wihtout imu
//note: the idea is fundamentaly flawed since loop closures can cause sudden jumps in frames and chunks.
//Consequently a kabsch trans that performs a jump or does not fit to other transformation does not need to be false.
//its the job of the high residual filter to take of such occurences
//However it is possible to implement a reduced version with an error threshold that depends on the time delay between frames
//This will not work for a cam that is held steadily but the filter is optional and for edge cases anyways.
void jumpingFilter(pairTransform &trans, shared_ptr<Frame> df)
{

    auto& matches = trans.filteredmatches;
    shared_ptr<Chunk> otherchunk = static_pointer_cast<Chunk>(trans.k1);
    shared_ptr<Chunk> currentChunk = static_pointer_cast<Chunk>(trans.k2);
    pandia_integration::integrationlock.lock();
    Eigen::Vector6d duplicateFrameDof = df->getWorlddofs(); //this is equal to chunktoworld for new chunk!
    pandia_integration::integrationlock.unlock();

    //note that this is the first frame position for the new chunk. Chunktransform is identity here
    Eigen::Matrix4d kabschPos = otherchunk->chunktoworldtrans * trans.getTransformationFrom(currentChunk, otherchunk);
    Eigen::Vector6d kabschDof = MattoDof(kabschPos);

    Eigen::Vector3d diff = kabschDof.tail(3) - duplicateFrameDof.tail(3);

    // angle between two dof rotation
    Eigen::Matrix4d kabschRot = getRz(kabschDof(2)) * getRy(kabschDof(1)) * getRx(kabschDof(0));
    Eigen::Matrix4d duplicateFrameRot = getRz(duplicateFrameDof(2)) * getRy(duplicateFrameDof(1)) * getRx(duplicateFrameDof(0));
    Eigen::Vector4d vec = Eigen::Vector4d(0,0,1,0);
    // double t_scalar = (kabschRot * vec).dot(duplicateFrameRot * vec);
    double t_angle = std::acos((kabschRot * vec).dot(duplicateFrameRot * vec)); // acos values are 0 to pi

    double t_err0 = 0.2;
    double t_err = min(t_err0* (1 + 1.0/60.0 * (currentChunk->structureIndex - otherchunk->structureIndex)),3 * t_err0);
    double t_errAngle0 = 0.31415926536; //18 degrees
    double t_errAngle = min(t_errAngle0 * (1 + 1.0 / 60.0 * (currentChunk->structureIndex - otherchunk->structureIndex)), 3 * t_errAngle0);

    if (diff.norm() > t_err || t_angle > t_errAngle) 
    {
        matches = vector<match>();
        trans.set = false;
        if (t_angle > t_errAngle) {
            open3d::utility::LogDebug("!!!!!!Jumping Filter used (error angle surpassed), angle was {} rad or {} degrees \n",
             t_angle, toDegrees(t_angle));
        }
        else {
            open3d::utility::LogDebug("Jumping Filter used. Error was {} \n", diff.norm());
        }
    }
}
