#ifndef MPIC_NRD_3D_H
#define MPIC_NRD_3D_H

#include "NodeRegistrationDeciderInterface.h"
#include "karst_slam/scanMerging/trajectoryPose.h"
#include "mrpt/poses/CPointPDFGaussian.h"
#include "karst_slam/obs/ObservationMSISBeam.h"
#include "karst_slam/obs/ObservationOdometry.h"
#include "karst_slam/scanMerging/surfaceDatum.h"
#include "karst_slam/scanMerging/surfaceTrainingData.h"

// Forward declarations
namespace karst_slam
{
    namespace sensors
    {
        class MSIS_primary;
        class MSIS_secondary;
    }
}

// Node register class based on a MSIS (sonar).
// Simply generate a new node at each full scan
namespace karst_slam{namespace nrd{

/** Convenient buffer containing all data of a scan */
struct scanDataBuffer  
{
    void reset()
    {
        verticalSonarData.clear();
        horizontalSonarData.clear();
        odometries.clear();
        nHorizontalStep = 0;
    }

    void print() const
    {
        std::cout << "verticalSonarData timestamps   : " << std::endl;
        for(const karst_slam::obs::ObservationMSISBeamPtr& p : verticalSonarData)
            std::cout << p->timestamp << " , " << std::endl;
        std::cout << "horizontalSonarData timestamps : " << std::endl;
        for(const karst_slam::obs::ObservationMSISBeamPtr& p : horizontalSonarData)
            std::cout << p->timestamp << " , " << std::endl;
        std::cout << "odometries                     : " << std::endl;
        for(const karst_slam::obs::ObservationOdometryPtr& p : odometries)
            std::cout << p->timestamp << " , " << p->pose_pdf.mean << std::endl;
        std::cout << "nHorizontalStep                : " << nHorizontalStep << std::endl;
        std::cout << "startTimestamp                 : " << startTimestamp  << std::endl;
        std::cout << "startPoseRelativeToPrevScanFirstFrame : " << startPoseRelativeToPrevScanFirstFrame.mean << std::endl;
    }

    std::vector<karst_slam::obs::ObservationMSISBeamPtr> verticalSonarData; //!< vector of beam data from the vertical sonar
    std::vector<karst_slam::obs::ObservationMSISBeamPtr> horizontalSonarData; //!< vector of beam data from the horizontal sonar
    std::vector<karst_slam::obs::ObservationOdometryPtr> odometries; //!< vector of odometry data
    int nHorizontalStep = 0; //!< Horizontal sonar step counter (!= horizontalSonarData.size() when some beams provide no valid ranges)
    uint64_t startTimestamp; //!< First time stamp
    mrpt::poses::CPose3DPDFGaussian startPoseRelativeToPrevScanFirstFrame; //!< Relative pose between the first frame of the previous and current scan
};

/** Basic vertical sonar data */
struct verticalSonarData
{
    uint64_t timestamp;
    double range;
    double yaw;
};

/** Basic horizontal sonar data */
struct horizontalSonarData
{
    uint64_t timestamp;
    std::vector<double> ranges; //!< Possibly more than one range per sonar beam (due to wide-beam)
    double yaw; 
    double alpha = 1.; //!< Parameter of elevation angle Beta distribution (Init: uniform distribution)
    double beta = 1.; //!< Parameter of elevation angle Beta distribution (Init: uniform distribution)
};

/** Contains processed scan data (ie with interpolated poses t each measure, inputs (s, yaw) for the surface learning, 
 * etc.) 
 */
struct fullScanProcessedData
{
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> drPoses;
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> interpolated_drPoses_v;
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> interpolated_drPoses_h;
    karst_slam::scanMerging::surfaceTrainingData verticalScanData;
    std::vector<horizontalSonarData> horizontalScanData;
    mrpt::poses::CPose3DPDFGaussian startPoseRelativeToPrevScanFirstFrame;
    mrpt::poses::CPose3DPDFGaussian startFrame_globalPose;
    uint64_t refFrame_timestamp;

    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor> verticalSonarPoints;
    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor> offRangeVerticalSonarPoints;
    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor> horizontalSonarPoints;
};

/** NRD to be used with MpIC (cf papers in doc) */
class MpIC_NRD3D: public virtual NodeRegistrationDeciderInterface
{
public:

    // Public functions
    //////////////////////////////////////////////////////////////
    using InfMat = mrpt::math::CMatrixFixedNumeric<double,
                                                   pose_pdf_t::state_length,
                                                   pose_pdf_t::state_length>;
    /**\brief Typedef for accessing methods of the RangeScanRegistrationDecider
     * parent class.
     */
    using decider_t   = MpIC_NRD3D; /**< self type - Handy typedef */
    /**\brief Node Registration Decider */
    using parent_t    = NodeRegistrationDeciderInterface;
    /**\}*/

    /** Class constructor */
    explicit MpIC_NRD3D(const std::string& configFile,
                        const std::string& verticalSonarLabel,
                        const std::string& horizontalSonarLabel,
                        uint64_t startTimestamp);

    explicit MpIC_NRD3D(const mrpt::utils::CConfigFile& configFile,
                        const std::string& verticalSonarLabel,
                        const std::string& horizontalSonarLabel,
                        uint64_t startTimestamp);

    /** Class destructor */
    ~MpIC_NRD3D();

    bool checkInit_()const override{return true;}

    void setMSISPtr(const std::shared_ptr<karst_slam::sensors::MSIS_primary>& msis);
    void setMSISPtr_noLoc(const std::shared_ptr<karst_slam::sensors::MSIS_secondary>& msis);

    inline std::shared_ptr<fullScanProcessedData> getFullScanProcessedData(){return m_lastProcessedScan;}
    
protected:
    bool checkRegistrationCondition() override;

    void updateStateFromObservationMSISBeam(const karst_slam::obs::ObservationMSISBeamPtr &obs) override;

    // Tmp accumulate odometry using given relative pose.
    // ToDo : receive DVL + IMU and compute odometry (here dead reckoning localization) with kalman filter
    void updateStateFromObservationOdometry(const karst_slam::obs::ObservationOdometryPtr &obs) override;

    void processFinishedScan(pose_pdf_t& scan_refFrameGlobalPose, 
                             pose_pdf_t& scan_fromRefFrameToEndFramePose,
                             pose_pdf_t& scan_fromPrefNodeToCurNode);

    void interpolateDeadReckoningPoses(mrpt::utils::TNodeID newNodeID,
                                       mrpt::poses::CPose3DPDFGaussian& refFrame_local,
                                       mrpt::poses::CPose3DPDFGaussian& refFrameToLastOdo,
                                       uint64_t& refFrame_timestamp);
    karst_slam::scanMerging::surfaceTrainingData processVerticalSonarData(const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& interpolated_drPoses,
                                                                          const mrpt::poses::CPose3D& refFrame_local, 
                                                                          mrpt::utils::TNodeID nodeId,
                                                                          Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& offRangeVerticalSonarPoints,
                                                                          Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& localPtsInRefFrame);
    std::vector<horizontalSonarData> processHorizontalSonarData(const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& interpolated_drPoses,
                                                                const mrpt::poses::CPose3D& refFrame_local,
                                                                mrpt::utils::TNodeID nodeId,
                                                                Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& localPtsInRefFrame);


    bool registerNewNodeAtEnd(const pose_pdf_t& scan_refFrameLocalPose,
                              const pose_pdf_t& scan_fromRefFrameToLastOdo,
                              const pose_pdf_t& scan_fromPrefNodeToCurNode);

    std::shared_ptr<karst_slam::sensors::MSIS_primary> m_msis; //!< pointer of class managing the main sonar (horizontal)
    std::shared_ptr<karst_slam::sensors::MSIS_secondary> m_msisMap; //!< pointer of class managing the secondary sonar (vertical)

    std::string m_verticalSonarLabel; //!< Name of the vertical sonar 
    std::string m_horizontalSonarLabel; //!< Name of the horizontal sonar 

    //pose_pdf_t m_since_prev_beam_pdf;

    scanDataBuffer m_currentScanDataBuffer; //!< Data concerning the current scan
    scanDataBuffer m_finishedScanDataBuffer; //!< Data of the (previous) finished scan

    std::shared_ptr<fullScanProcessedData> m_lastProcessedScan; 
    std::vector<mrpt::poses::CPose3D> m_lastBodyPosesatVerticalSonarMeasures_wrtFirstFrame; //!< Used by ERD for surface estimation

    bool m_scanFinished; //!< True when the scan has finished (but not yet registrable)
    bool m_finishedScanRegistrable; //!< True when the last finished is registrable

    pose_pdf_t m_lastFinishedScanPosePDF;

    /** Relative pose from the previous scan reference frame to the next scan first frame 
     * (first frame of scan are set to be the last odometry pose before the first scan horizontal sonar measurement)
     */
    pose_pdf_t m_previousScanRefFrameToPenultimateOdo; 
    bool m_firstScan = true;

    /** Number of steps to create a node
      * By default, one complete scan corresponds to 200 steps */
    int m_nHorizontalSonarStepForNodeCreation = 200;

    bool m_getOffRangeVerticalSonarLimit; //!< If true, save the max range point corresponding to each vertical sonar pose

    bool m_useOffRangePts; //!< If true, also include the off-range data in the scans (considered as "censored data")
};
}} // end namespaces

#endif // MPIC_NRD_3D_H