#ifndef MSIS_SECONDARY_H
#define MSIS_SECONDARY_H

#include "karst_slam/sensors/MSIS.h"

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

// ToDo : See how to avoid copying matrices when going from beams_t to MSIS_scan
// Currently use double but allows templating for a float version ?
/** Class managing an acoustic sonar
 * "Secondary" means that it is not used to estimate the robot displacement 
 * (vertical sonar in the pb dealt in the papers in /doc) 
 */ 
class MSIS_secondary : public MSIS //public Sensor //public mrpt::utils::COutputLogger
{

public:
    using beams_t = std::map<int, karst_slam::obs::beamPoints>;
    using cov_t   = mrpt::math::CMatrixDouble33;

    /**
     * @brief Constructor without configuration file. Caution : Need to set the parameters with setParams() or loadParams().
     * @param deviceName Device name
     */
    MSIS_secondary(const std::string& deviceName);
    ~MSIS_secondary();

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

    /**
     * Generate a full scan with accumulated beams when the primary scan finished its own full scan
     * 
     * Old function not used anymore
     */ 
    //void generateScan(mrpt::poses::CPose3DPDFGaussian &referenceFrameLocalPose_SensorFrame,
    //                  const mrpt::poses::CPose3DPDFGaussian& referenceFrameGlobalPose_BodyFrame,
    //                  const mrpt::poses::CPose3DPDFGaussian& referenceFrameToEndFrame_BodyFrame);

protected:
    void reset() override;
    std::map<int, mrpt::poses::CPose3DPDFGaussian> computeBeamPosesWRTReferenceFrame(mrpt::poses::CPose3DPDFGaussian &referenceFrameLocalPose) override;
};
}} // end namespaces

#endif // MSIS_SECONDARY_H
