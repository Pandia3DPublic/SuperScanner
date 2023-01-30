#pragma once
#include "core/Frame.h"
#include "core/KeypointUnit.h"
#include "core/threadCom.h"
#include "utils/coreutil.h"
#include "c_keypoint.h"
//note: gpu orb keypoints variant is slower than cpu on small images, faster for full size but still too slow
// timings small image (640x480): cpu 6-8 ms, gpu 10-15 ms (sometimes even 25ms)
// timings full res image (2048x1536): cpu 50 ms, gpu 20-30 ms ->too slow
//note that this always gets called when the frames are generated
//depth is in host space!
template <typename T> //to pass mirror view object with unknown type
void generateOrbKeypoints(std::shared_ptr<Frame> f, const cv::Mat &cvColor, T depth)
{ //take opencv mat seperately
    if (g_nkeypoints == -1)
        open3d::utility::LogError("Number of keypoints is not set!\n");

    cv::Mat orbDescriptors;
    std::vector<cv::KeyPoint> orbKeypoints;

    auto kpd = cv::ORB::create(g_nkeypoints);
    kpd->detectAndCompute(cvColor, cv::noArray(), orbKeypoints, orbDescriptors);

    Eigen::Matrix3d &intr = g_intrinsic.intrinsic_matrix_;
    // Eigen::Matrix3d &intr = g_fullIntr.intrinsic_matrix_; //2048x1536 for kinect
    int nvalid = 0;
    std::vector<int> validentries(orbKeypoints.size());
    //check if depth is non zero
    for (int i = 0; i < orbKeypoints.size(); i++)
    {
        auto &k = orbKeypoints[i];
#ifdef ENABLEASSERTIONS
        if ((int)k.pt.x > depth.extent(1) || (int)k.pt.y > depth.extent(0))
        {std::cout << "Warning memory violation in genKps.h!! x " << k.pt.x << "  y " << k.pt.y << " \n";}
#endif
        // if (depth((int)round(k.pt.y), (int)round(k.pt.x)) != 0) //todo is this mem save? no!
        if (depth((int)k.pt.y, (int)k.pt.x) != 0)
        {
            validentries[nvalid] = i;
            nvalid++;
        }
    }
    f->orbKeypoints.reserve(nvalid);
    f->orbDescriptors = cv::Mat::zeros(nvalid, g_nOrb, CV_8U);

    for (int i = 0; i < nvalid; i++)
    {
        auto &k = orbKeypoints[validentries[i]];
        // float x = round(k.pt.x);
        // float y = round(k.pt.y);
        // int x = round(k.pt.x);
        // int y = round(k.pt.y);
        int x = (int)k.pt.x;
        int y = (int)k.pt.y;
        float d = (float)depth(y, x) / 1000.0;
        // std::cout << x << " " << y << std::endl;
        // std::cout << "depth " << d << std::endl;
        // auto d = (float)depth(x,y) / 1000.0;

        // fill keypoints
        c_keypoint ke;
        //use non rounded image coords for keypoint calculation, should be more precise, but is not todo
        // ke.p = Eigen::Vector4d((k.pt.x - intr(0, 2)) * d / intr(0, 0), (k.pt.y - intr(1, 2)) * d / intr(1, 1), d, 1);
        // Eigen::Vector3d p = imgCoordstoPoint(Eigen::Vector2i(x, y),d, intr);
        // ke.p = Eigen::Vector4d(p(0),p(1),p(2),1);
        ke.p = Eigen::Vector4d((x - intr(0, 2)) * d / intr(0, 0), (y - intr(1, 2)) * d / intr(1, 1), d, 1);
        ke.des.reserve(32);
        // build the filtered descriptor matrix
        for (int j = 0; j < 32; j++)
        {
            f->orbDescriptors.at<uchar>(i, j) = orbDescriptors.at<uchar>(validentries[i], j);
            ke.des.push_back(orbDescriptors.at<uchar>(validentries[i], j));
        }
        f->orbKeypoints.push_back(ke);
    }
}
