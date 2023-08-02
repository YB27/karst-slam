#include "karst_slam/simulation/simulationTrajectory_base.h"
#include <mrpt/random.h>

using namespace std;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace mrpt::utils;
using namespace mrpt::poses;

simulationTrajectory_base::simulationTrajectory_base(const CConfigFile &cfg)
{
    m_var_xyz   = cfg.read_double("Simulation", "varianceTrajNoise_xyz", 1e-3, true);
    m_var_euler = cfg.read_double("Simulation", "varianceTrajNoise_euler", 1e-4, true);
    m_cov = Eigen::Matrix<double, 6, 6>::Identity();
    m_cov(0, 0) = m_var_xyz;
    m_cov(1, 1) = m_var_xyz;
    m_cov(2, 2) = m_var_xyz;
    m_cov(3, 3) = m_var_euler;
    m_cov(4, 4) = m_var_euler;
    m_cov(5, 5) = m_var_euler;

    // Now the seed is fixed in the .ini file
    //mrpt::random::randomGenerator.randomize(0);
    m_noisyTraj = cfg.read_bool("Simulation", "noisyTraj", true, true);
    m_useAbsoluteDepthPitchRoll = cfg.read_bool("Simulation", "useDepthPitchRollAbsoluteValues", false, true);
    m_absoluteZ_var     = cfg.read_double("Simulation", "variance_depth", 0.,true);
    m_absoluteAngle_var = DEG2RAD(cfg.read_double("Simulation", "variance_imu_angles", 0.,true));

    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

void simulationTrajectory_base::fixDepthPitchRollVariances(CPose3DPDFGaussian& pose)
{
    for (int i = 0; i < 6; i++)
    {
        pose.cov(i, 2) = 0.;
        pose.cov(2, i) = 0.;
        pose.cov(i, 4) = 0.;
        pose.cov(4, i) = 0.;
        pose.cov(i, 5) = 0.;
        pose.cov(5, i) = 0.;
    }
    pose.cov(2, 2) = m_absoluteZ_var;
    pose.cov(4, 4) = m_absoluteAngle_var;
    pose.cov(5, 5) = m_absoluteAngle_var;
}

void simulationTrajectory_base::fixDepthPitchRoll(trajectoryPose<CPose3DPDFGaussian>& trajPose)
{
    // Draw samples for depth, pitch and roll
    if(m_noisyTraj)
    { 
        double angleStd = sqrt(m_absoluteAngle_var);
        double z_sample  = mrpt::random::randomGenerator.drawGaussian1D(trajPose.pose_gt.mean.z(), sqrt(m_absoluteZ_var)),
            yaw_sample   = mrpt::random::randomGenerator.drawGaussian1D(trajPose.pose_gt.mean.yaw(), angleStd), 
            pitch_sample = mrpt::random::randomGenerator.drawGaussian1D(trajPose.pose_gt.mean.pitch(), angleStd),
            roll_sample  = mrpt::random::randomGenerator.drawGaussian1D(trajPose.pose_gt.mean.roll(), angleStd); 
        trajPose.pose.mean.m_coords[2] = z_sample;
        trajPose.pose.mean.setYawPitchRoll(yaw_sample, pitch_sample, roll_sample);
    }

    fixDepthPitchRollVariances(trajPose.pose);
    fixDepthPitchRollVariances(trajPose.pose_gt);
}
