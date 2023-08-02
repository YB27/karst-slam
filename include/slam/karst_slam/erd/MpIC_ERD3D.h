#ifndef MPIC_ERD3D_H
#define MPIC_ERD3D_H

#include "karst_slam/internal.h"
#include "EdgeRegistrationDeciderInterface.h"
#include "karst_slam/obs/ObservationMSIS_scan.h"
#include "karst_slam/scanMerging/surfaceGP.h"
#include "karst_slam/scanMerging/dataForScanMerging.h"

// Forward declarations
namespace karst_slam
{
    namespace icp{class PICP;}
    namespace obs{class ObservationMSIS_scan;}
    namespace nrd{class fullScanProcessedData;}
}

/**
 *  Edge Registration Decider (ERD) based on Mechanically Scanned Imaging Sonars (MSIS).
 *  Constraints (relative pose) are obtained using a probabilistic Iterative Correspondence (pIC) algorithm.
 *  (stochastic version of ICP)
 */
namespace karst_slam {namespace erd {
class MpIC_ERD3D : public EdgeRegistrationDeciderInterface
{
public:
    using parent_t = EdgeRegistrationDeciderInterface;
    using nodes_to_scans_t = std::map<mrpt::utils::TNodeID, std::vector<karst_slam::obs::ObservationMSIS_scanPtr>>;

    explicit MpIC_ERD3D(const std::string& configFile);
    explicit MpIC_ERD3D(const mrpt::utils::CConfigFile& configFile);

    virtual ~MpIC_ERD3D() override;

    /**
     * Update the state of this ERD with newly received data 
     */ 
    bool updateState(const mrpt::obs::CActionCollectionPtr &action,
                     const mrpt::obs::CSensoryFramePtr &observations,
                     const mrpt::obs::CObservationPtr &observation,
                     mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr()) override;

    bool checkInit_() const override;

    void loadParams(const mrpt::utils::CConfigFile& cfg);
    void printParams() const;
    void getDescriptiveReport(std::string* report_str) const;

    inline void setLastProcessedScan(const std::shared_ptr<karst_slam::nrd::fullScanProcessedData>& scan){m_previousScan = m_lastScan; 
                                                                                                          m_lastScan     = scan;}
    inline const karst_slam::scanMerging::scanMergingResults& getLastScanMergingResults() const 
    {return m_lastSuccessiveScanMergingResults;}
    inline const std::vector<std::vector<karst_slam::scanMerging::pointThetaSimu>>& getLastHorizontalSonarPoints_thetaVals() const 
    {return m_lastHorizontalSonarPoints_thetaVals;}
    inline const std::vector<karst_slam::scanMerging::horizontalSonarMeasure>& getLastHorizontalSonarMeasures() const 
    {return m_lastHorizontalSonarMeasures;}
    inline const scanMerging::ellipticCylinder& getLastEllipticCylinder()const 
    {return m_lastEllipticCylinder;}
    inline const scanMerging::surfaceTrainingData& getTrainingData()const
     {return m_trainingData;}

    // TODO : Sonar poses should be accessible from elsewhere !
    inline const mrpt::poses::CPose3D& getHorizontalSonarPose()const 
    {return m_params.horizontalSonarParams.sensorPose.mean;}

private:
    /**
     * Express one scan data points in global frame
     * 
     * @param refFrame_globalPose Scan reference frame express in global frame
     * @param scanData scan data expressed in the scan reference frame
     * @param verticalPoints_globalFrame [out] vertical sonar measured points expressed in the global frame
     * @param bodyPosesAtPointMeasurement_globalFrame [out] Global Robot body poses at each sonar measurements
     * @param bodyPosesAtPointMeasurement_ts [out] Timestamps at each sonar measurements
     */ 
    void scanDataFromLocalToGlobalFrame(const mrpt::poses::CPose3DPDFGaussian& refFrame_globalPose,
                                        const std::shared_ptr<nrd::fullScanProcessedData>& scanData,
                                        std::vector<mrpt::poses::CPointPDFGaussian>& verticalPoints_globalFrame,
                                        std::vector<mrpt::poses::CPose3DPDFGaussian>& bodyPosesAtPointMeasurement_globalFrame,
                                        std::vector<uint64_t>& bodyPosesAtPointMeasurement_ts);

    /**
     * Express a pair of scan data points in global frame
     * 
     * @param firstScanFirstFrame_globalPose Global pose of the first frame of the first scan
     * @param secondScan_firstFrame_globalPose Global pose of the first frame of the second scan
     * @param verticalPoints_globalFrame [out] vertical sonar measured points expressed in the global frame
     * @param bodyPosesAtPointMeasurement_globalFrame [out] Global Robot body poses at each sonar measurements
     * @param bodyPosesAtPointMeasurement_ts [out] Timestamps at each sonar measurements
     */ 
    void pairScanDataInGlobalFrame(const mrpt::poses::CPose3DPDFGaussian& firstScanFirstFrame_globalPose,
                                   const mrpt::poses::CPose3DPDFGaussian& secondScan_firstFrame_globalPose,
                                   std::vector<mrpt::poses::CPointPDFGaussian>& verticalPoints_globalFrame,
                                   std::vector<mrpt::poses::CPose3DPDFGaussian>& bodyPosesAtPointMeasurement_globalFrame,
                                   std::vector<uint64_t>& bodyPosesAtPointMeasurement_ts);

    void saveHorizontalSonarEstimatedData(const scanMerging::scanMergingResults& res);
    void saveBodyPoses(const std::vector<mrpt::poses::CPose3DPDFGaussian>& bodyPoses,
                       const std::vector<uint64_t>& timestamps,
                       const std::string& fileName) const;
    void saveRefFramesTimestamps(uint64_t firstScan_refFrame_timestamp,
                                 uint64_t secondScan_refFrame_timestamp) const;
    /**
     * Save data to files to be further used for scan merging (ie elevation angle estimation) in sonar_gp.py
     * and then scan matching in sonarPICP_simulation.py
     * 
     * @param horizontalData horizontal sonar data for which elevation angles will be estimated
     * @param bodyPoses vector of robot poses pdf
     * @param timestamps 
     * @param firstScan_refFrame_timestamp
     * @param secondScan_refFrame_timestamp
     */ 
    void saveToFiles_forPythonScripts(const scanMerging::surfaceValidationData& horizontalData,
                                      const std::vector<mrpt::poses::CPose3DPDFGaussian>& bodyPoses,
                                      const std::vector<uint64_t>& timestamps,
                                      uint64_t firstScan_refFrame_timestamp,
                                      uint64_t secondScan_refFrame_timestamp) const;

    /**
     * Process the data coming from the horizontal sonar to then be used for scan merging (ie elevation angle estimation)
     * 
     * @param firstFrame_globalPose Global pose of the first frame of the current scan
     * @param scan Data of the scan
     * @param horizontalSonarPoints_thetaVals [out] Sampled points along each measured beams (sample on the elevation angle)
     * @param horizontalSonarMeasures [out] Data of the horizontal sonar measures
     * @param scan_idx 
     */ 
    void processHorizontalSonarData(const mrpt::poses::CPose3DPDFGaussian& firstFrame_globalPose,
                                    const std::shared_ptr<nrd::fullScanProcessedData>& scan, 
                                    std::vector<std::vector<scanMerging::pointThetaSimu>>& horizontalSonarPoints_thetaVals,
                                    std::vector<scanMerging::horizontalSonarMeasure>& horizontalSonarMeasures,
                                    int scan_idx);

    /**
     * Express the vertical sonar data in the Global frame 
     * 
     * @param prevScanData Vertical sonar data of the previous scan
     * @param lastScanData Vertical sonar data of the current (last) scan  
     * @param startFrame_globalPose Global pose of the first frame of the previous scan
     * @param Scans_relativePose Relative pose between the two scans reference frames
     * @return scanMerging::surfaceTrainingData Data of pair of scans in the Global frame
     */
    scanMerging::surfaceTrainingData verticalDataInGlobalFrame(const scanMerging::surfaceTrainingData& prevScanData,
                                                               const scanMerging::surfaceTrainingData& lastScanData,
                                                               const mrpt::poses::CPose3DPDFGaussian& startFrame_globalPose,
                                                               const mrpt::poses::CPose3DPDFGaussian& Scans_relativePose);
    /**
     * Check if the current pair of scans can be registered as an edge in the PoseGraph
     * Compute both edge with previous scan and loop closure if any was detected
     * 
     * @todo Loop closure not yet implemented / tested 
     */
    virtual void checkRegistrationCondition() override;
    
    /**
     * Compute the relative pose pdf between the two last scan
     */
    void edgeWithPreviousScan();

    /**
     * Get candidate nodes for loop closure
     * 
     * @return std::set<mrpt::utils::TNodeID> candidate node ids
     */
    std::set<mrpt::utils::TNodeID> getNodesToCheckForICP_LC() const;

    nodes_to_scans_t m_nodes_to_scans; //!< map linking node id to sonar data 
    scanMerging::surfaceTrainingData m_trainingData; //!< Vertical sonar data used to learn the environment surface
    std::shared_ptr<nrd::fullScanProcessedData> m_lastScan; //!< Current (last) scan data
    std::shared_ptr<nrd::fullScanProcessedData> m_previousScan; //!< Previous scan data
    scanMerging::scanMergingResults m_lastSuccessiveScanMergingResults; //!< Last scan merging results (elevation angles pdf estimation)
    std::vector<std::vector<scanMerging::pointThetaSimu>> m_lastHorizontalSonarPoints_thetaVals; //!< Sampled points on beam of the last scan horizontal data
    std::vector<scanMerging::horizontalSonarMeasure> m_lastHorizontalSonarMeasures; //!< Horizontal sonar data of the last scan
    scanMerging::ellipticCylinder m_lastEllipticCylinder; //!< Prior elliptic cylinder
    scanMerging::surfaceGP m_surfaceGP; //!< Used to compute the environment surface estimation with Gaussian Process
    bool m_isLearningMeanFunc; //!< Testing. If true, learn the prior function for environment surface (instead of elliptic cylinder)
    double m_sphereRadius_densityForKernelTemperature; //!< Testing. Radius used to compute point density (used as temperature parameters for kernels of the Gaussian Process)
};
}} // end namespaces

#endif // MPIC_ERD3D_H
