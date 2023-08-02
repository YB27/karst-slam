#ifndef DEF_SIMULATION_TRAJECTORY_LINEAR_FORWARD_H
#define DEF_SIMULATION_TRAJECTORY_LINEAR_FORWARD_H

#include "simulationTrajectory_base.h"

namespace karst_slam{namespace simulation {
/** Simulate a simple trajectory */
class simulationTrajectory_linearForward : public simulationTrajectory_base
{
public:

    explicit simulationTrajectory_linearForward(const mrpt::utils::CConfigFile& cfg);

    karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> getPose(bool updateOdo,
                           uint64_t timeStamp,
                           const karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>& trajPrevPose = 
                           karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>()) override;
};
}}
#endif //DEF_SIMULATION_TRAJECTORY_LINEAR_FORWARD_H 