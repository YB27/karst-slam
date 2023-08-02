#include "karst_slam/simulation/simulationTrajectory_loopCentered.h"
#include "karst_slam/simulation/observationSimulator_sonar.h"
#include <mrpt/random.h>

using namespace std;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::math;

simulationTrajectory_loopCentered::simulationTrajectory_loopCentered(const CConfigFile& cfg) : simulationTrajectory_base(cfg)
{
    m_type = LOOP_CENTERED;
}

trajectoryPose<CPose3DPDFGaussian> simulationTrajectory_loopCentered::getPose(bool updateOdo,
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

    // Move virtually the robot along its local x axis (forward)
    CPose3D robotVirtualPose = trajPrevPose.pose_gt.mean; 

    // Center the robot in the gallery
    // Here, simply take the barycenter but use the deformable vitual zone (DVZ)
    // First, ray-trace in the current plane
    TPoint3D verticalSonarPositionOnRobot(m_verticalSonarSimulator->getSensorPoseOnRobot().mean.m_coords[0],
                                          m_verticalSonarSimulator->getSensorPoseOnRobot().mean.m_coords[1],
                                          m_verticalSonarSimulator->getSensorPoseOnRobot().mean.m_coords[2]);
    //TPoint3D barycenter = m_verticalSonarSimulator->sectionBarycenter(robotVirtualPose) - 
    //                      verticalSonarPositionOnRobot;
    TPoint3D barycenter = m_verticalSonarSimulator->sectionBarycenter(robotVirtualPose);
    barycenter.x += 0.1;
    MRPT_LOG_DEBUG_STREAM("[simulationTrajectory_loopCentered::getPose] Barycenter next section    : " << barycenter);

    // Note that odometry must be expressed in the current robot frame
    CPose3D incr_mean(barycenter);
    // Should move forward
    if(incr_mean.x() < 0)
        MRPT_LOG_ERROR_STREAM("[simulationTrajectory_loopCentered::getPose] Robot moved backward !");

    CPose3DPDFGaussian incr(incr_mean, incr_cov);
    trajPose.pose_gt = trajPrevPose.pose_gt + incr;
    if(updateOdo)
        m_drPose_GT.push_back(incr);

    if(m_noisyTraj)
    {
        CPose3DPDFGaussian incr_noisy;
        incr.drawSingleSample(incr_noisy.mean);
        incr_noisy.cov = incr.cov;

        cout <<"GT odo    : " << incr.mean << endl;
        cout <<"Noisy odo : " << incr_noisy.mean << endl;

        trajPose.pose = trajPrevPose.pose + incr_noisy;
        if(updateOdo)
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