#ifndef DEF_SIMULATION_ENGINE_H
#define DEF_SIMULATION_ENGINE_H

#include "karst_slam/scanMerging/dataForScanMerging.h"
#include "karst_slam/scanMerging/curvilinearAbscissa_SE3.h"
#include "karst_slam/scanMerging/interpolationSE3.h"
#include "simulationTrajectory_linearForward.h"
#include "simulationTrajectory_customFile.h"
#include "simulationTrajectory_loopCentered.h"
#include "simulationTrajectory_loop_circle.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/COutputLogger.h>
#include <Eigen/Core>
#include <memory>

namespace mrpt{namespace poses{ class CPointPDFGaussian;}}

namespace karst_slam{namespace simulation{

// Fwd dcl
class observationSimulator_sonar;
class iglModel;

/** Sonar-related data which are buffered to then be used for sonar measure simulation */
struct sonarDataForBuffer
{
    sonarDataForBuffer(uint64_t timeStamp_,
                       double angle_)
    {
        timeStamp = timeStamp_;
        angle = angle_;
    }
    uint64_t timeStamp; //!< timestamp of the measure to be simulated
    double angle; //!< Sonar rotation angle (yaw)
    karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> posePDFAtMeasurement; //!< 3D pose pdf of the sonar at this beam
};

/** Parameters used in the simulation */
struct simulationParams
{
   simulationParams() = default;
   simulationParams(const mrpt::utils::CConfigFile& cfg)
   {
       nAngleStep              = cfg.read_int("Simulation", "nAngleStep"  , 200, true);
       anglePerStep            = 2.f*M_PI/(float)nAngleStep;
       nStepToRun              = cfg.read_int("Simulation", "nStepToRun"  , 400, true);
       nScanDataBetweenOdoData = cfg.read_int("Simulation", "nScanDataBetweenOdoData",1, true);
       interpolationType       = (karst_slam::scanMerging::INTERPOLATION_TYPE) cfg.read_int("General", "interpolationType",0, true);

       saveFolder              = cfg.read_string("General", "dataFolder", "", true);

       isLearnMeanFunc         = cfg.read_bool("GaussianProcess", "learnMeanFunction", false, true); 
   }

   int nAngleStep; //!< Number of step for a complete turn of the horizontal sonar
   float anglePerStep; //!< Angle per step (in rad)
   int nStepToRun; //!< Number of step executed in the simulation for one full (horizontal) scan
   int nScanDataBetweenOdoData; //!< Number of vertical scan beam received during the interval between two odometry data
   karst_slam::scanMerging::INTERPOLATION_TYPE interpolationType; //!< Interpolation type (LINEAR or SPLINE)
   bool isLearnMeanFunc; //!< If true, the prior environment surface is learned (eg with MLP) (just a test). Otherwise it is an elliptic cylinder as in papers. 

   std::string saveFolder; //!< Folder where to save simulated data
};

/** Main class for simulation */
class simulationEngine : public mrpt::utils::COutputLogger
{
public:
    /**
     * Constructor
     * 
     * @param cfg Configuration file
     * @param verticalSonarSimulator Vertical sonar simulator
     * @param horizontalSonarSimulator Horizontal sonar simulator
     */
    simulationEngine(const mrpt::utils::CConfigFile& cfg,
                     const std::shared_ptr<observationSimulator_sonar>& verticalSonarSimulator,
                     const std::shared_ptr<observationSimulator_sonar>& horizontalSonarSimulator);

    /**
     * Generate simulated data in case of a closed karst 3D model. Used to test the pICP for loop closure
     * @return res simulated data
     */ 
    //scanMerging::dataForScanMerging_loopClosure generateSimulatedData_LoopClosure();

    /**
     * Generate simulated data in case of a closed karst 3D model. Used to test the pICP for loop closure
     * Unlike generateSimulatedData_LoopClosure(), here compute also the intermediate scans
     * 
     * @param surfaceEstimator 
     * @return res simulated data
     */ 
    scanMerging::dataForScanMerging_loopClosure generateSimulatedData_LoopClosure_allScans(const std::shared_ptr<scanMerging::surfaceGP>& surfaceEstimator);

    /**
     * Simulate the measures for at least two successives scans and then infer the elevation angles for each horizontal sonar scans
     * The data is then saved to files and ready for scan matching 
     * 
     * @param res_scanGrp contains data for 2 full-scans (for both vertical + horizontal sonar)
     * @param surfaceEstimator used to compute estimate the surface environment with GaussianProcess
     * @param pairScanIdx index of the pair of scans
     * @param nScanToRun number of full scan to simulate
     * @param previousScan If any, the previously simulated scan
     * @return the data for scan merging of the last scan
     */
    std::shared_ptr<scanMerging::dataForScanMerging> generateScan(std::shared_ptr<scanMerging::dataForScanMerging>& res_scanGrp,
                                                                  const std::shared_ptr<scanMerging::surfaceGP>& surfaceEstimator,
                                                                  uint pairScanIdx, 
                                                                  int nScanToRun = 2,
                                                                  std::shared_ptr<scanMerging::dataForScanMerging> previousScan=nullptr);
    
    /** 
     * Generate the dead-reckoning robot trajectory along a full-scan while buffering the time/pose at which sonar simulations will be done
     * 
     * @param curScanIdx  current scan index
     * @param verticalSonarTimeStamps Buffered data for simulation of vertical sonar measures
     * @param horizontalSonarTimeStamps Buffered data for simulation of horizontal sonar measures
     * @return robot trajectory
     */
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> generateTrajectoryDR(int curScanIdx,
        std::vector<sonarDataForBuffer>& verticalSonarTimeStamps,
        std::vector<sonarDataForBuffer>& horizontalSonarTimeStamps);

    /** 
     *  Interpolate the robot poses pdf at each vertical and horizontal sonar measurements
     *  (odometry data (from IMU/DVL) and sonar measurements are asynchronous hence the interpolation)
     * 
     *  @param drPoses dead-reckoning robot poses (odometry composition)
     *  @param verticalSonarDataForSimu Buffered data for simulation of vertical sonar measures
     *  @param horizontalSonarDataForSimu Buffered data for simulation of horizontal sonar measures
     */
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> posesPDFInterpolationAtMeasurements(const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& drPoses,
                                                                                                                                     const std::vector<sonarDataForBuffer>& verticalSonarDataForSimu,
                                                                                                                                     const std::vector<sonarDataForBuffer>& horizontalSonarDataForSimu) const;
    /**
     * Save the curvilinear Abscissa in a file
     * 
     * @param name Name of the file
     * @param curvilinearAbscissa Vector of curvilinear abscissa
     */
    void saveCurvilinearAbscissa(const std::string& name,
                                 const std::vector<double> &curvilinearAbscissa) const;

    /**
     * Save Odometry values in a file
     * 
     * @param name Name of the file
     * @param odometry Vector of odometry
     */
    void saveOdometry(const std::string& name,
                      const std::vector<mrpt::poses::CPose3DPDFGaussian>& odometry) const;

    /** Simple criteria based on distance to detect a loop closure */
    bool loopClosure(const mrpt::poses::CPose3D& robotGlobalPose,
                     double distanceThreshold);
    
    /** Set the callback function called each time a pair of scans has been generated */
    inline void setCallbackAtScanGeneration(const std::function<void(std::shared_ptr<scanMerging::dataForScanMerging>, scanMerging::scanMergingResults)>& callback)
    {m_callbackAtScanGeneration = callback;}

private:
    /** Compute the orthogonal proj of point in the cylinder axis (for curvilinear abscissa) */
    double projectionAbscissaOnGuidingCurve(const mrpt::poses::CPointPDFGaussian& point_globalFrame, 
                                            const Eigen::Vector3d& cylinderAxis);

    /** Save data on files (to be used by sonarPICP_simulation.py) */
    void saveSimulationData(const std::shared_ptr<scanMerging::dataForScanMerging>& data);

    /** Simulate data for one scan 
     * 
     * @param curScanIdx simulated scan index
     * @param cylinderAxis Axis of the prior cylinder (environment surface estimation)
     * @return data for scan merging (horizontal sonar elevation angle estimation)
    */
    std::shared_ptr<karst_slam::scanMerging::dataForScanMerging> simulateDataForOneScan(int curScanIdx,
                                                                                        const Eigen::Vector3d& cylinderAxis);
    
    /**
     * Simulate vertical sonar measurements 
     * 
     * @param data buffered data used as input for simulation
     * @param simData [out] results of the simulation are put in this ptr 
     */ 
    void simulateVerticalSonarMeasurements(const std::vector<sonarDataForBuffer>& data,
                                           std::shared_ptr<karst_slam::scanMerging::dataForScanMerging> simData);

    /**
     * Simulate one vertical sonar measurement 
     * 
     * @param data buffered datum used as input for simulation
     * @param simData [out] results of the simulation are put in this ptr 
     */ 
    void simulateVerticalSonarMeasurement(const sonarDataForBuffer& data,
                                          std::shared_ptr<karst_slam::scanMerging::dataForScanMerging> simData);

    /**
     * Simulate horizontal sonar measurements 
     * 
     * @param data buffered data used as input for simulation
     * @param curScanIdx 
     * @param simData [out] results of the simulation are put in this ptr 
     */ 
    void simulateHorizontalSonarMeasurements(const std::vector<sonarDataForBuffer>& data,
                                             int curScanIdx,
                                             std::shared_ptr<karst_slam::scanMerging::dataForScanMerging> simData);
    
    /**
     * Simulate one horizontal sonar measurement
     * 
     * @param data buffered data used as input for simulation
     * @param curScanIdx 
     * @param simData [out] results of the simulation are put in this ptr 
     */ 
    void simulateHorizontalSonarMeasurement(const sonarDataForBuffer& data,
                                            int curScanIdx,
                                            std::shared_ptr<karst_slam::scanMerging::dataForScanMerging> simData);

    double rotateToAlignRobotInKarst_loopClosure(const Eigen::Vector3d& cylinderAxis,
                                               const mrpt::poses::CPose3D& lastRobotPose);

    //void moveUntilLoopClosure(karst_slam::scanMerging::trajectory<mrpt::poses::CPose3DPDFGaussian>& bodyPoses_deadreckoning_loop);


    simulationParams m_params; //!< Simulation parameters
    std::shared_ptr<observationSimulator_sonar> m_verticalSonarSimulator; //!< Vertical sonar simulator
    std::shared_ptr<observationSimulator_sonar> m_horizontalSonarSimulator; //!< Horizontal sonar simulator

    Eigen::Matrix<double,6,6> m_cov_small; //!< Small covariance matrix (used to avoid singularities)
    
    std::shared_ptr<simulationTrajectory_base> m_trajectoryGenerator; //!< Object creating trajectories (expected)
    karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian> m_curGlobalPose_deadreckoning; //!< Current robot pose estimated by dead-reckoning
    double m_curCurvilinearAbscissa = 0.; //!< Current curvilinear abscissa
    uint64_t m_timeStamp; //!< Current timestamp 
    
    std::function<void(std::shared_ptr<scanMerging::dataForScanMerging>, scanMerging::scanMergingResults)> m_callbackAtScanGeneration;
};
}} // end namespaces

#endif // DEF_SIMULATION_ENGINE_H
