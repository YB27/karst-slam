#ifndef DEF_SIMULATION_TRAJECTORY_CUSTOM_FILE_H
#define DEF_SIMULATION_TRAJECTORY_CUSTOM_FILE_H

#include "simulationTrajectory_base.h"

namespace karst_slam{namespace simulation {

/** Simulate a trajectory based on a custom file */
class simulationTrajectory_customFile : public simulationTrajectory_base
{
public:
    
    explicit simulationTrajectory_customFile(const mrpt::utils::CConfigFile& cfg);

    /**
     * Load the trajectory from a file. Only for the CUSTOM_FILE flag.
     * 
     * @param fileName File
     */
    void loadFrom(const std::string& fileName);

    void loadAbsoluteDepthPitchRoll(const std::string& fileName);

    karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> getPose(bool /* update */,
                           uint64_t timeStamp,
                           const karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>& trajPrevPose = 
                           karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>()) override;

protected:
    std::vector<mrpt::poses::CPose3DPDFGaussian> m_customLocalIncrements;
    std::vector<double> m_absoluteZ_samples;
    std::vector<double> m_absolutePitch_samples;
    std::vector<double> m_absoluteRoll_samples;
    uint m_lastIdx = 0;

};
}}
#endif // DEF_SIMULATION_TRAJECTORY_CUSTOM_FILE_H