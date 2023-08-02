#ifndef DEF_SIMULATION_TRAJECTORY_LOOP_CIRCLE_H
#define DEF_SIMULATION_TRAJECTORY_LOOP_CIRCLE_H

#include "simulationTrajectory_base.h"

// Fwd dcl
namespace karst_slam{namespace simulation{class observationSimulator_sonar;}}

namespace karst_slam{namespace simulation {

/** Simulate a trajectory following a circle (for simulation in the karst donut) */
class simulationTrajectory_loop_circle : public simulationTrajectory_base
{
public:
    explicit simulationTrajectory_loop_circle(const mrpt::utils::CConfigFile& cfg);

    karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> getPose(bool updateOdo,
                           uint64_t timeStamp,
                           const karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>& trajPrevPose = 
                           karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>()) override;

    inline void setVerticalSonarSimulator(const std::shared_ptr<observationSimulator_sonar>& verticalSonarSimulator)
    {m_verticalSonarSimulator = verticalSonarSimulator;}

protected:
    std::shared_ptr<observationSimulator_sonar> m_verticalSonarSimulator; ///< Sonar simulator used to compute the centered trajectory
};
}}
#endif // DEF_SIMULATION_TRAJECTORY_LOOP_CENTERED_H