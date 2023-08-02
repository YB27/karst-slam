#ifndef MSISVIEWER_H
#define MSISVIEWER_H

#include "SensorViewer.h"
#include "karst_slam/typedefs.h"
#include "karst_slam/mrpt_poses_frwd.h"
#include <mrpt/utils/TColor.h>
#include <Eigen/Core>

#define INVALID_SCANID -1

// Forward declarations
namespace mrpt
{
    namespace opengl{class COpenGLScenePtr;}
    namespace obs{class CObservationPtr;}
    namespace utils
    {
        class CConfigFile;
        class TColor;
    }
}
namespace karst_slam{namespace obs{class ObservationMSIS_scan;}}

namespace karst_slam{namespace gui{
/** Convenient struct regrouping objects names related to the same pdf point cloud */
struct MSISViewer_aux
{
    MSISViewer_aux() = default;
    MSISViewer_aux(mrpt::utils::TNodeID id,
                   const std::string& sensorLabel)
    {
        std::string id_str = std::to_string(id);
        pointCloudName = "scan_pointCloud_"   + sensorLabel + "_" + id_str;
        ellipsoidsName = "scan_ellipsoids_"   + sensorLabel + "_" + id_str;
        refFrameAxisName   = "scan_refFrameAxis_" + sensorLabel + "_" + id_str;
    }

    std::string pointCloudName;
    std::string ellipsoidsName;
    std::string refFrameAxisName;
};

/** Viewer for displaying information from MSIS (acoustic sonar) */
class MSISViewer : public SensorViewer
{
public:
    MSISViewer(const std::string& sensorLabel, MainViewer *mainViewer = nullptr);
    ~MSISViewer() = default;

    /** 
     * Derived function for rendering from SensorViewer
     * @todo Pb : inputs are not adequated here (hence renderScan). The structure should also be refactored !
     */
    void render(mrpt::opengl::COpenGLScenePtr& scene, const mrpt::obs::CObservationPtr& scan, const mrpt::poses::CPose3D &refFrame) override{};

    /**
     * Main function for rendering measures by an acoustic sonar scan in 3D
     * 
     * @param scene
     * @param localScanPts Matrix of points expressed in the scan frame
     * @param refFrame Frame in which the points (localScanPts) are expressed
     * @param nodeId id of the node in the PoseGraph to which the scan is attached  (ie when the scan has been taken)
     */ 
    void renderScan(mrpt::opengl::COpenGLScenePtr& scene, 
                    const points_mat_t& localScanPts, 
                    const mrpt::poses::CPose3D &refFrame,
                    mrpt::utils::TNodeID nodeId);

    void toggleEllipsoidsVisualization();
    void toggleScansVisualization();

protected:
    void init_(mrpt::opengl::COpenGLScenePtr& scene,
              const mrpt::utils::CConfigFile& cfg) override;
    void initKeystrokes(const mrpt::utils::CConfigFile& cfg) override;

    void initNewScanObjects(scanID id,
                            mrpt::opengl::COpenGLScenePtr& scene,
                            MSISViewer_aux& aux,
                            mrpt::opengl::CPointCloudPtr& pointCloud,
                            mrpt::opengl::CSetOfObjectsPtr& ellipsoids,
                            mrpt::opengl::CSetOfObjectsPtr& refFrameAxis);

    void getScanObjects(scanID id,
                        mrpt::opengl::COpenGLScenePtr& scene,
                        MSISViewer_aux& aux,
                        mrpt::opengl::CPointCloudPtr& pointCloud,
                        mrpt::opengl::CSetOfObjectsPtr& ellipsoids,
                        mrpt::opengl::CSetOfObjectsPtr& refFrameAxis);

    std::map<scanID,MSISViewer_aux> m_aux; //!< Vector containing 3D object names per scan Id
    bool m_displayRefFrameAxis = true; //!< If true, display the ref frame axis
    mrpt::utils::TColor m_color; //!< Default color of scans (last and penultimate used in the last scan scan matching are draw with different colors)
    mrpt::utils::TColor m_lastScanColor; //!< Color of the penultimate scan
    mrpt::utils::TColor m_curScanColor; //!< Color of the last scan
    double m_ellipsoidsQuantile = 3.; //!< Quantile used to draw the uncertainty ellipses
    float m_pointSize = 1.f; //!< Size of points
    float m_oldScansPointSize = 1.f; //!< Size of points for scans before the last and penultimate ones
    bool m_ellipsoidsVisible = false; //!< If true, draw the uncertainty ellipses
    bool m_scansVisible = true; //!< If true, display the scans   
    scanID m_lastScanID = INVALID_SCANID; //!< Id of the last scan
    scanID m_curScanID = INVALID_SCANID; //!< Id of the penultimate scan
};
}} // end namespaces
#endif // MSISVIEWER_H
