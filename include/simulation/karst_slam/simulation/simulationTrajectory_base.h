#ifndef DEF_SIMULATION_TRAJECTORY_BASE_H
#define DEF_SIMULATION_TRAJECTORY_BASE_H

#include <memory>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/COutputLogger.h>
#include "karst_slam/scanMerging/trajectoryPose.h"

namespace karst_slam{namespace simulation {

/** Enum for differents types of simulated trajectories */
enum TRAJECTORY_TYPE{LINEAR_CONST_SPEED_X = 0, //!< Straight line at constant speed
                     CUSTOM_FILE = 1, //!< Use a trajectory defined in a file
                     LOOP_CENTERED = 2, //!< Trajectory based on section barycenter + cylinder fitting for rotation
                     LOOP_CIRCLE = 3 //!< Simple trajectory following a circle
                     };

class simulationTrajectory_base : public mrpt::utils::COutputLogger
{
public:
    /** Default constructor */
    simulationTrajectory_base() = default;

    virtual ~simulationTrajectory_base() = default;

    /**
     * Constructor
     * 
     * @param cfg Configuration file
     */
    explicit simulationTrajectory_base(const mrpt::utils::CConfigFile& cfg);

    /**
     * Get the pose at a given index for the trajectory,
     * depending on its type given by the TRAJECTORY_TYPE enum.
     * 
     * @param updateOdo If true, update the dead reckoning pose
     * @param timeStamp 
     * @param trajPrevPose Previous pose 
     * @return 3D pose
     *
     */
    virtual karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> getPose(bool updateOdo,
                                                                                             uint64_t timeStamp,
                                                                                             const karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>& trajPrevPose = 
                                                                                             karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>()) = 0;

    inline const std::vector<mrpt::poses::CPose3DPDFGaussian>& getDRPose() const {return m_drPose;}
    inline const std::vector<mrpt::poses::CPose3DPDFGaussian>& getDRPose_GT() const {return m_drPose_GT;}
    inline const mrpt::math::CMatrixDouble66& getOdoCov() const {return m_cov;}
    inline const TRAJECTORY_TYPE getType() const {return m_type;}

protected:
    /** Fix the values which are absolute and not relative (depth from pression sensor, pitch/roll from IMU) 
     * 
     * @param trajPose Pose to be fixed
     */
    void fixDepthPitchRoll(karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>& trajPose);

    /** Fix the variance values which are absolute and not relative (depth from pression sensor, pitch/roll from IMU) 
     * 
     * @param pose Pose to be fixed
     */
    void fixDepthPitchRollVariances(mrpt::poses::CPose3DPDFGaussian& pose);

    TRAJECTORY_TYPE m_type; //!< Trajectory type (see TRAJECTORY_TYPE enum)
    double m_var_xyz; //!< variance of noise. Only for noisy trajectories
    double m_var_euler; //!< variance of noise. Only for noisy trajectories
    mrpt::math::CMatrixDouble66 m_cov; //!< Covariance. Only for noisy trajectories
    std::vector<mrpt::poses::CPose3DPDFGaussian> m_drPose; //!< odometry. Only for noisy trajectories
    std::vector<mrpt::poses::CPose3DPDFGaussian> m_drPose_GT; //!< Ground truth odometry
    bool m_useAbsoluteDepthPitchRoll; //!< Consider z, pitch and roll values as absolute (--> no propagation of error)
    std::vector<double> m_absoluteZ; //!< Z absolute values
    std::vector<double> m_absolutePitch; //!< pitch absolute values
    std::vector<double> m_absoluteRoll; //!< roll absolute values
    double m_absoluteZ_var; //!< Z absolute variance
    double m_absoluteAngle_var; //!< Angle(Pitch/roll) absolute variance 
    bool m_noisyTraj; //!< If true, add noise 
};
}}
#endif // DEF_SIMULATION_TRAJECTORY_BASE_H