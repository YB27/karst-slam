#ifndef MSIS_H
#define MSIS_H

#include "karst_slam/mrpt_matrix_frwd.h"
#include "karst_slam/mrpt_poses_frwd.h"
#include "Sensor.h"
#include "karst_slam/obs/ObservationMSIS_scan.h"
#include <map>
#include <vector>
#include <memory>

// Forward declarations
namespace mrpt
{
    namespace utils{class CConfigFile;}
}

// Defined in Scan.h
namespace karst_slam
{
    namespace obs
    {
        class ObservationMSIS_scanPtr;
        class beamPoints;
    }
    namespace sensors
    {
        class CosSin;
        class CartesianCov_lut;
    }
}

namespace karst_slam{namespace sensors{
/** Parameters / characteristics of acoustic sonars */
struct MSISParams
{
    mrpt::poses::CPose3DPDFGaussian sensorPose; //!< Sonar pose relative to the robot body frame
    mrpt::poses::CPose3DPDFGaussian sensorPoseInv; //!< Precomputed inverse of sensorPose
    mrpt::poses::CPose3DQuatPDFGaussian sensorPoseQuat; //!< SensorPose but in 3D + Quaternion
    mrpt::poses::CPose3DQuatPDFGaussian sensorPoseQuatInv; //!< Precomputed inverse of sensorPoseQuat
    float horizontalBeamWidth = 1.f; //!< Horizontal beam width in radian
    float verticalBeamWidth = 1.f; //!< Vertical beam width in radian
    float maxRange = 20.f; //!< Maximum range in meter
    float minRange = 0.5f; //!< Minimum range in meter. See the spec of the MSIS or look at directly into data to determine it
    int minIntensityBin = 0; //!< Minimum valid intensity bin (depends on min range)
    int nIntensityBins = 50; //!< Number of intensity bins
    float intensityStep = 0.4f; //!< Range represented by a bin in meter
    int nAngleStep = 200; //!< Number of angle step (ie number of beams) to define a full scan
    int intensityThreshold = 20; //!< Minimum threshold value for intensity 
    int minDistanceFilter = 3; //!< Min. distance (in number of bins) between two intensity bin to be considered as coming from two different objects
    float rangeStd = 0.4f; //!< Standard deviation for range meas. (m)
    float angle_yaw_Std = 1.6875 /* 360/200 */; //!< Standard deviation for angular meas. (deg)
    float angle_pitch_Std; //!< standard deviation for pitch angle (deg)
    mrpt::math::CMatrixDouble33 sphericalLocalCovariance; //!< Covariance matrix of spherical coordinates
};

// We should not accumulated "null" odometry as it increases the covariance
// -> use the "isNull" parameter to skip null odometry
/** Helper for computing beam relative pose to previous beam */
struct beamRelativePosePdf
{
    beamRelativePosePdf()
    {
        posePdf = mrpt::poses::CPose3DPDFGaussian();
        isNull = true;
    }

    beamRelativePosePdf(const mrpt::poses::CPose3DPDFGaussian& posePdf_,
                        bool isNull_)
    {
        posePdf = posePdf_;
        isNull = isNull_;
    }

    mrpt::poses::CPose3DPDFGaussian posePdf;
    bool isNull;
};

// ToDo : See how to avoid copying matrices when going from beams_t to MSIS_scan
// Currently use double but allows templating for a float version ?
/** Generic class for defining an acoustic sonar 
 *  
 *  Note : old class made before the original MpIC method was made
 *  So this class (and its derivated classes MSIS_primary and MSIS_secondary) are in a large part unused
*/
class MSIS : public Sensor 
{

public:
    using beams_t = std::map<int, karst_slam::obs::beamPoints>;
    using cov_t = mrpt::math::CMatrixDouble33;

    /**
     * @brief Constructor without configuration file. Caution : Need to set the parameters with setParams() or loadParams().
     * @param deviceName Device name
     */
    MSIS(const std::string& deviceName);
    ~MSIS();

    /**
     * Add a new beam to the current sonar scan 
     * 
     * @param beam measured beam to be added
     * @param accumulatedOdoSinceLastBeam Accumulated robot odometry pdf since the previous added beam
     * @param currentGlobalPose Current global pose pdf of the robot 
     * @param sincePrevScanRefFramePdf Pose pdf relative to the previous full scan refence frame 
     * @param hasAccumulatedOdoSinceLastBeam True if new odometry data has been received since the last beam 
     */ 
    virtual void addBeam(const karst_slam::obs::ObservationMSISBeamPtr& beam,
                         const mrpt::poses::CPose3DPDFGaussian &accumulatedOdoSinceLastBeam,
                         const mrpt::poses::CPose3DPDFGaussian &currentGlobalPose,
                         const mrpt::poses::CPose3DPDFGaussian &sincePrevScanRefFramePdf,
                         bool hasAccumulatedOdoSinceLastBeam) = 0;

    /**
     * Filter the vector of intensities to keep only subset corresponding to object measurements
     * 
     * @param intensities
     * @return vector of filtered ranges in meters 
     */ 
    std::vector<double> filterIntensitiesToRanges(const std::vector<int> &intensities);

    void loadParams(const mrpt::utils::CConfigFile& cfg) override;

    /**
     * Process a vector of intensities to a set of measured points in cartesian coords
     * 
     * @param intensities
     * @param beamAngleIdx idx of the processed beam (from which comes the vector of intensities)
     * @return measured points
     */ 
    karst_slam::obs::beamPoints fromIntensitiesToCartesianCoords(const std::vector<int>& intensities, 
                                                                 int beamAngleIdx);
    
    /** \overload */
    std::vector<mrpt::poses::CPointPDFGaussian> fromIntensitiesToCartesianCoords(const std::vector<int>& intensities, 
                                                                                 double beamAngle, 
                                                                                 std::vector<double>& ranges);
    
    //mrpt::poses::CPointPDFGaussian fromIntensitiesToGlobalCoords(const std::vector<int>& intensities, 
    //                                                             double beamAngle); 

    inline const MSISParams& getParams() const {return m_params;}
    inline double getAnglePerStep()const{return m_anglePerStep;}
    inline const mrpt::poses::CPose3DPDFGaussian& getSensorPose() const{return m_params.sensorPose;}
    inline const mrpt::poses::CPose3DPDFGaussian& getSensorPoseInv() const{return m_params.sensorPoseInv;}
    inline const mrpt::poses::CPose3DQuatPDFGaussian& getSensorPoseQuat() const{return m_params.sensorPoseQuat;}
    inline const mrpt::poses::CPose3DQuatPDFGaussian& getSensorPoseInvQuat() const{return m_params.sensorPoseQuatInv;}

    inline karst_slam::obs::ObservationMSIS_scanPtr getLastScan() const {return m_lastScan;}

    void setParams(const MSISParams& params);
    void setLastScanAsRegistered(mrpt::utils::TNodeID nodeId);
    inline bool lastScanRegistered() const{return m_lastScanRegistered;}
    inline bool hasCompletedFullScan()const{return m_hasCompletedFullScan;}

    void dumpParamsToConsole()const;
    static void dumpBeamsToConsole(const beams_t& beams);
    static void dumpScanToConsole(const karst_slam::obs::ObservationMSIS_scan& scan);

protected:
    virtual void init();

    /** Precompute LUT for cos and sin values for the beam angles */
    void initCosSinLUT();

    /** Init the local spherical covariance matrix for measured points */
    void initSphericalLocalCovariance();

    /** Init the local cartesian covariance matrix for measured points */
    void initCartesianLocalCovLUT();

    /** 
     * Compute the jacobian of 3D pose-point composition
     */
    mrpt::math::CMatrixFixedNumeric<double, 3, 6> computePosePointCompositionJacobionWRTPose(const double& lx, const double& ly, const double& lz,
                                                                                             const double &yaw, const double &pitch, const double &roll);

    /** Express the poses at which each beam has been measured relative to the scan reference frame */
    virtual std::map<int, mrpt::poses::CPose3DPDFGaussian> computeBeamPosesWRTReferenceFrame(mrpt::poses::CPose3DPDFGaussian& referenceFrameLocalPose) = 0;

    /** Compute the covariance matrix of a point in the scan reference frame 
     * 
     * @param pointCov point covariance in the scan first frame
     * @param lx x-coord of the point
     * @param ly y-coord of the point
     * @param lz z-coord of the point
     * @param poseFromIc reference frame pose
     * @return covariance matrix of the point expressed in the scan reference frame
    */
    cov_t computePointCovarianceInRefFrame(const cov_t& pointCov,
                                           const double& lx, const double& ly, const double& lz,
                                           const mrpt::poses::CPose3DPDFGaussian& poseFromIc);

    // Add small eps for the int conversion to get the right round (ie not (int)0.9999 -> 0)
    inline int getBeamAngleIdx(double angle) const 
    {return std::round(0.01 + angle/m_anglePerStep);}

    /** Convert bin index to the corresponding range */
    inline double binToRange(int bin) const 
    {return (double)(bin + 1)*m_params.intensityStep;}

    virtual void reset();

    /** Compute points pdf relative to the scan reference frame
     *  It is then stored in m_currentScan.  
     * 
     * @param beamIdx points pdf from this beam index to express in the scan reference frame
     * @param beamPoseWRTRefFrame pose of the beam (beamIdx) relative to the scan reference frame
    */
    void computePointsPDFInRefFrame(int beamIdx, const mrpt::poses::CPose3DPDFGaussian &beamPoseWRTRefFrame);

    beams_t m_currentScan; //!< Contains all data points sorted per beam
    karst_slam::obs::ObservationMSIS_scanPtr m_lastScan; //!< Last full scan. Can have several ranges for a same angle

    std::vector<beamRelativePosePdf> m_relativePoseToPrevBeam; //!< Contains the relative poses of beam i wrt beam i-1
    
    // Note : When the current full scan is completed (m_hasCompletedFullScan set to True), we have to wait for the next
    // odometry data to be able to interpolate the last robot trajectories in the completed scan.
    // Once this is done, we can register it in the poseGraph (m_lastScanRegistered set to True)
    // This implies that while waiting for the new odometry data, we already start the creation of the next scan. 
    bool m_lastScanRegistered; //!<  True if the last full scan has been registered in the poseGraph (!= completed)
    bool m_hasCompletedFullScan; //!< True when the current scan has been completed

    MSISParams m_params; //!< Parameters/Characteristics of the sonar

    double m_anglePerStep; //!< Angle between each sonar beam in radian
    std::map<int, CosSin> m_cosSinLUT; //!< LUT for cos/sin values of angles of beam wrt to the sonar reference frame
    std::map<int, CartesianCov_lut> m_cartesianCovLUT; //!< LUT for the covariance matrix of points in local cartesian coords
    mrpt::math::CMatrixDouble33 m_sphericalLocalCovariance; //!< Local covariance in spherical coords
    int m_curScanPoints; //!< Total number of points of the current scan in construction
    scanID m_lastScanId; //!< Id of the last finished scan
};
}} // end namespaces

#endif // MSIS_H
