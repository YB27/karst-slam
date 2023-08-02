#include "karst_slam/simulation/simulationTrajectory_linearForward.h"

using namespace std;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace mrpt::utils;
using namespace mrpt::poses;

simulationTrajectory_linearForward::simulationTrajectory_linearForward(const CConfigFile& cfg) : simulationTrajectory_base(cfg)
{
    m_type = LINEAR_CONST_SPEED_X;
}

trajectoryPose<CPose3DPDFGaussian> simulationTrajectory_linearForward::getPose(bool updateOdo,
                                                                               uint64_t timeStamp,
                                                                               const trajectoryPose<CPose3DPDFGaussian>& trajPrevPose)
{   
    trajectoryPose<CPose3DPDFGaussian> trajPose;
    trajPose.timeStamp = timeStamp;

    CPose3DPDFGaussian incr(CPose3D(0.2, /*-0.001*/ 0, 0, 0, 0, 0), m_cov);
    trajPose.pose_gt = trajPrevPose.pose_gt + incr;
    if (updateOdo)
        m_drPose_GT.push_back(incr);

    if (m_noisyTraj)
    {
        // This is the ground truth
        CPose3DPDFGaussian incr_noisy;
        incr.drawSingleSample(incr_noisy.mean);
        incr_noisy.cov = m_cov;
        // Avoid going back
        incr_noisy.mean.m_coords[0] = fabs(incr_noisy.mean.m_coords[0]);

        trajPose.pose = trajPrevPose.pose + incr_noisy;
        // This is the estimated pose
        if (updateOdo)
            m_drPose.push_back(incr_noisy);
    }
    else
    {
        trajPose.pose = trajPose.pose_gt;
        if(updateOdo)
            m_drPose.push_back(incr);
    }

    // Remove covariances with depth, pitch and rol
    if (m_useAbsoluteDepthPitchRoll)
        fixDepthPitchRoll(trajPose);

    return trajPose;
}

