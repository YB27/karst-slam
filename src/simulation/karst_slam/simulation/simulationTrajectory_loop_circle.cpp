#include "karst_slam/simulation/simulationTrajectory_loop_circle.h"
#include "karst_slam/simulation/observationSimulator_sonar.h"
#include <mrpt/random.h>

using namespace std;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

simulationTrajectory_loop_circle::simulationTrajectory_loop_circle(const CConfigFile& cfg) : simulationTrajectory_base(cfg)
{
    m_type = LOOP_CIRCLE;
}

trajectoryPose<CPose3DPDFGaussian> simulationTrajectory_loop_circle::getPose(bool updateOdo,
                                                                              uint64_t timeStamp,
                                                                              const trajectoryPose<CPose3DPDFGaussian> &trajPrevPose)
{
    trajectoryPose<CPose3DPDFGaussian> trajPose;
    trajPose.timeStamp = timeStamp;

    CMatrixDouble66 incr_cov;
    incr_cov(0,0) = m_var_xyz;
    incr_cov(1,1) = m_var_xyz;
    incr_cov(2,2) = m_absoluteZ_var;
    incr_cov(3,3) = m_absoluteAngle_var;
    incr_cov(4,4) = m_absoluteAngle_var;
    incr_cov(5,5) = m_absoluteAngle_var;

    double radius = 19.5, dx = 0.1;
    // Depth fix
    double yaw = asin(dx/radius);
    CPose3D incr_mean(dx, radius*(1. - cos(yaw)), 0., yaw, 0., 0.);

    CPose3DPDFGaussian incr(incr_mean, incr_cov);
    trajPose.pose_gt = trajPrevPose.pose_gt + incr;

    if(updateOdo)
        m_drPose_GT.push_back(incr);

    if(m_noisyTraj)
    {
        CPose3DPDFGaussian incr_noisy;
        incr.drawSingleSample(incr_noisy.mean);
        incr_noisy.cov = incr.cov;

        if(incr_noisy.mean[0] < 0)
            MRPT_LOG_ERROR_STREAM("[simulationTrajectory_loop_circle::getPose] Robot moved backward !");

        // Take in account that for x,y,yaw it's relative uncertainty whereas for z,pitch,roll it's absolute
        trajPose.pose = trajPrevPose.pose + incr_noisy;
        for(int i = 0 ; i < 6 ; i++)
        {
            trajPose.pose.cov(i,2) = 0.;
            trajPose.pose.cov(i,4) = 0.;
            trajPose.pose.cov(i,5) = 0.;
            trajPose.pose.cov(2,i) = 0.;
            trajPose.pose.cov(4,i) = 0.;
            trajPose.pose.cov(5,i) = 0.;
        }
        trajPose.pose.cov(2,2) = m_absoluteZ_var;
        trajPose.pose.cov(4,4) = m_absoluteAngle_var;
        trajPose.pose.cov(5,5) = m_absoluteAngle_var;
        if(updateOdo)
            m_drPose.push_back(incr_noisy);
    }
    else
    {
        trajPose.pose = trajPose.pose_gt;
        if(updateOdo)
            m_drPose.push_back(incr);
    }
    
    // Remove covariances with depth, pitch and roll
    if (m_useAbsoluteDepthPitchRoll)
        fixDepthPitchRoll(trajPose);

    
    return trajPose;
}