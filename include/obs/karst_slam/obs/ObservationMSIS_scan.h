#ifndef OBSERVATION_MSIS_SCAN_H
#define OBSERVATION_MSIS_SCAN_H

#include "ObservationMSISBeam.h"
#include "karst_slam/typedefs.h"
#include "karst_slam/mrpt_poses_frwd.h"
#include <mrpt/poses/CPointPDFGaussian.h>

// Forward declarations
namespace mrpt{namespace obs{class CObservation3DRangeScan;}}

/** Full scan in the reference frame
 */
namespace karst_slam{namespace obs{

DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE(ObservationMSIS_scan,CObservation,OBS_IMPEXP)
class OBS_IMPEXP ObservationMSIS_scan : public mrpt::obs::CObservation
{
    DEFINE_SERIALIZABLE(ObservationMSIS_scan)
public:
    using beams_t = std::map<int, beamPoints>;
    using points_mat_t = Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>;
     
    /** Constructor returning MRPT smart pointers using constructor with parameters */
    static ObservationMSIS_scanPtr Create(beams_t&& beamMap, int nPoints_,
                                          const mrpt::poses::CPose3DPDFGaussian& refFrameGlobalPoseAtCreation_,
                                          const mrpt::poses::CPose3DPDFGaussian& refFrameToEndFramePose_,
                                          const std::string& deviceName,
                                          scanID scanId);

    ObservationMSIS_scan() = default;

    ObservationMSIS_scan(beams_t&& beamMap, int nPoints_,
                         const mrpt::poses::CPose3DPDFGaussian &refFrameGlobalPoseAtCreation_,
                         const mrpt::poses::CPose3DPDFGaussian &refFrameToEndFramePose_,
                         const std::string& deviceName,
                         scanID scanId);

    /** Compose the scan observation (eg measured points) with the given pose (inplace)*/
    void composeWithPose(const mrpt::poses::CPose3D& pose);

    /** Compose the scan observation with the given pose and return a new scan observation */
    ObservationMSIS_scanPtr createComposeWithPose(const mrpt::poses::CPose3D &pose)const;

    /** Convert to 3Drange scan by dropping the covariance associated to each point */
    mrpt::obs::CObservation3DRangeScan convertTo3DRangeScan() const;

    void printScanData()const;

    inline void setNodeId(mrpt::utils::TNodeID nodeId){m_nodeId = nodeId;}
    inline mrpt::utils::TNodeID getNodeId()const{return m_nodeId;}

    inline mrpt::poses::CPointPDFGaussian getPointPose(int idx) const
    {
        return mrpt::poses::CPointPDFGaussian(mrpt::poses::CPoint3D(m_points(0,idx), m_points(1,idx), m_points(2,idx)),
                                               m_cov[idx]);
    }
    inline scanID getScanId()const{return m_scanId;}

    inline void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const {out_sensorPose = m_sensorPoseOnRobot;}
    inline void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override{m_sensorPoseOnRobot = newSensorPose;}
    void getDescriptionAsText(std::ostream& o) const override;

    points_mat_t m_points; //!< Matrix of points constituting the scan observation
    eigenAlignedVector<cov_t> m_cov; //!< covariance matrices of each observed points
    int m_nPoints; //!< Size of the point cloud
    /** Global pose of the scan reference frame AT the time it was created.
     * Warning : After graph optimization, the estimated pose will change and this value should not be used anymore.
     * Use the node pose on which this scan is attached.
     */
    mrpt::poses::CPose3DPDFGaussian m_refFrameGlobalPoseAtCreation;
    mrpt::poses::CPose3DPDFGaussian m_refFrameToEndFramePose; //!< Pose between the scan reference frame (middle of the scan) and its last frame
    mrpt::utils::TNodeID m_nodeId = INVALID_NODEID;

protected:
    mrpt::poses::CPose3D m_sensorPoseOnRobot;//!< Pose of the scan sensor relative to the robot body frame
    scanID m_scanId;
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( ObservationMSIS_scan,mrpt::obs::CObservation,OBS_IMPEXP)
}}// end namespace

#endif // OBSERVATION_MSIS_SCAN_H
