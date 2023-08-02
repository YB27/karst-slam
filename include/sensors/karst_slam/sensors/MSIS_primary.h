#ifndef MSIS_PRIMARY_H
#define MSIS_PRIMARY_H

#include "karst_slam/sensors/MSIS.h"

// Forward declarations
namespace mrpt
{
    namespace utils{class CConfigFile;}
}

// Defined in Scan.
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

// ToDo : See how to avoid copying matrices when going from beams_t to MSIS_scan
// Currently use double but allows templating for a float version ?
/** Class managing an acoustic sonar
 * "Primary" means that it is used to estimate the robot displacement 
 * (horizontal sonar in the pb dealt in the papers in /doc) 
 */ 
class MSIS_primary : public MSIS
{

public:
    using beams_t = std::map<int, karst_slam::obs::beamPoints>;
    using cov_t   = mrpt::math::CMatrixDouble33;

    /**
     * @brief Constructor without configuration file. Caution : Need to set the parameters with setParams() or loadParams().
     * @param deviceName Device name
     */
    MSIS_primary(const std::string& deviceName);
    ~MSIS_primary();

    /**
     * Add a new beam to the current sonar scan 
     * 
     * @param beam measured beam to be added
     * @param accumulatedOdoSinceLastBeam Accumulated robot odometry pdf since the previous added beam
     * @param currentGlobalPose Current global pose pdf of the robot 
     * @param sincePrevScanRefFramePdf Pose pdf relative to the previous full scan refence frame 
     * @param hasAccumulatedOdoSinceLastBeam True if new odometry data has been received since the last beam 
     */ 
    void addBeam(const karst_slam::obs::ObservationMSISBeamPtr& beam,
                 const mrpt::poses::CPose3DPDFGaussian &accumulatedOdoSinceLastBeam,
                 const mrpt::poses::CPose3DPDFGaussian &currentGlobalPose,
                 const mrpt::poses::CPose3DPDFGaussian &sincePrevScanRefFramePdf,
                 bool hasAccumulatedOdoSinceLastBeam) override;
    
    inline const mrpt::poses::CPose3DPDFGaussian& getLastScan_refFrameGlobalPoseAtCreation() const
    {return m_lastScan->m_refFrameGlobalPoseAtCreation;}
    inline const mrpt::poses::CPose3DPDFGaussian& getFromRefFrameToEndFramePose() const
    {return m_lastScan->m_refFrameToEndFramePose;}
    inline const mrpt::poses::CPose3DPDFGaussian& getFromPrevScanRefFramePose() const
    {return m_sincePrevScanRefFramePdf;}
    inline const mrpt::poses::CPose3DPDFGaussian& getRefFrameLocalPose() const
    {return m_refFrameLocalPose;}


protected:
    void init() override;

    int getOffsetBeamAngleIdx(int beamAngleIdx) const;

    /** Express the poses at which each beam has been measured relative to the scan reference frame */
    std::map<int, mrpt::poses::CPose3DPDFGaussian> computeBeamPosesWRTReferenceFrame(mrpt::poses::CPose3DPDFGaussian& referenceFrameLocalPose) override;

    /** Generate the full scan once all required beams have been received */
    void generateFullScan();

    mrpt::poses::CPose3DPDFGaussian m_firstBeamGlobalPoseInBodyFrame; //!< Global pose of the first beam frame
    mrpt::poses::CPose3DPDFGaussian m_fromLocalFrameToEndFramePosePdf; //!< Pose pdf between the scan reference frame and the frame of the last beam
    mrpt::poses::CPose3DPDFGaussian m_sincePrevScanRefFramePdf; //!< Pose pdf relative to the previous full scan refence frame 
    mrpt::poses::CPose3DPDFGaussian m_refFrameLocalPose; //!< Reference frame of the scan 
    int m_beamOffsetIdx; //!< Offset on the beam indexes (in case we consider that a scan do not start at the first beam ie at the rotation angle 0)
    int m_centerIdx; //!< Idx of the beam corresponding to the reference frame (ie near the center of the scan)
};
}} // end namespaces

#endif // MSIS_PRIMARY_H
