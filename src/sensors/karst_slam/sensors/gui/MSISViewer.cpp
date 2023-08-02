#include "karst_slam/obs/ObservationMSIS_scan.h"
#include "karst_slam/sensors/gui/MSISViewer.h"
#include "karst_slam/gui/utils.h"
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/TColor.h>

using namespace std;
using namespace mrpt::opengl;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace karst_slam;
using namespace karst_slam::gui;
using namespace karst_slam::obs;

MSISViewer::MSISViewer(const std::string& sensorLabel, MainViewer* mainViewer) : SensorViewer(sensorLabel, mainViewer),
    m_color(mrpt::utils::TColor(255.,0.,0.))
{}


void MSISViewer::init_(COpenGLScenePtr& scene,
                      const CConfigFile& cfg)
{
    MRPT_START
    MRPT_UNUSED_PARAM(scene);
    const string prefix = "MSISScan_" + m_sensorLabel + "_";

    // Point clouds
    m_pointSize         = cfg.read_float(m_configFileSection, prefix + "pointSize", 1.f, true);
    m_oldScansPointSize = cfg.read_float(m_configFileSection, prefix + "oldScansPointSize", 1.f, true);
    unsigned char pointColor_r = cfg.read_int(m_configFileSection, prefix + "pointColor_r", 0,true);
    unsigned char pointColor_g = cfg.read_int(m_configFileSection, prefix + "pointColor_g", 167,true);
    unsigned char pointColor_b = cfg.read_int(m_configFileSection, prefix + "pointColor_b", 255,true);
    unsigned char lastScan_pointColor_r = cfg.read_int(m_configFileSection, prefix + "lastScan_pointColor_r", 255,true);
    unsigned char lastScan_pointColor_g = cfg.read_int(m_configFileSection, prefix + "lastScan_pointColor_g", 0,true);
    unsigned char lastScan_pointColor_b = cfg.read_int(m_configFileSection, prefix + "lastScan_pointColor_b", 0,true);
    unsigned char curScan_pointColor_r = cfg.read_int(m_configFileSection, prefix + "curScan_pointColor_r", 0,true);
    unsigned char curScan_pointColor_g = cfg.read_int(m_configFileSection, prefix + "curScan_pointColor_g", 255,true);
    unsigned char curScan_pointColor_b = cfg.read_int(m_configFileSection, prefix + "curScan_pointColor_b", 0,true);
    m_color = TColor(pointColor_r, pointColor_g, pointColor_b);
    m_lastScanColor = TColor(lastScan_pointColor_r,lastScan_pointColor_g,lastScan_pointColor_b);
    m_curScanColor  = TColor(curScan_pointColor_r, curScan_pointColor_g, curScan_pointColor_b);
    m_displayRefFrameAxis = cfg.read_bool(m_configFileSection, prefix + "refFrameAxis",true,true);

    MRPT_END
}

void MSISViewer::initNewScanObjects(scanID id,
                                    COpenGLScenePtr &scene,
                                    MSISViewer_aux& aux,
                                    CPointCloudPtr &pointCloud,
                                    CSetOfObjectsPtr &ellipsoids,
                                    CSetOfObjectsPtr& refFrameAxis)
{
    // Change the color of the previous "last scan" to the normal color and size
    if(m_lastScanID != INVALID_SCANID)
    {
        const MSISViewer_aux& aux_lastScan = m_aux[m_lastScanID];
        CPointCloudPtr cloud = static_cast<CPointCloudPtr>(scene->getByName(aux_lastScan.pointCloudName));
        cloud->setColor_u8(m_color);
        cloud->setPointSize(m_oldScansPointSize);
        static_cast<CSetOfObjectsPtr>(scene->getByName(aux_lastScan.ellipsoidsName))->setColor_u8(m_color);
    }

    m_lastScanID = m_curScanID;
    m_curScanID = id;

    aux = MSISViewer_aux(id,m_sensorLabel);
    pointCloud = CPointCloud::Create();
    pointCloud->setPointSize(m_pointSize);
    pointCloud->enablePointSmooth();
    pointCloud->setColor_u8(m_curScanColor);
    pointCloud->setName(aux.pointCloudName);
    scene->insert(pointCloud);
    pointCloud->setVisibility(m_scansVisible);

    ellipsoids = CSetOfObjects::Create();
    ellipsoids->setColor_u8(m_curScanColor);
    ellipsoids->setName(aux.ellipsoidsName);
    scene->insert(ellipsoids);
    ellipsoids->setVisibility(m_ellipsoidsVisible);

    if(m_displayRefFrameAxis)
    {
        refFrameAxis = mrpt::opengl::stock_objects::CornerXYZ();
        refFrameAxis->setName(aux.refFrameAxisName);
        scene->insert(refFrameAxis);
    }

    // Change the color of the previous "cur scan" to the "last scan" color
    if(m_lastScanID != INVALID_SCANID)
    {
        const MSISViewer_aux& aux_lastScan = m_aux[m_lastScanID];
        static_cast<CPointCloudPtr>(scene->getByName(aux_lastScan.pointCloudName))->setColor_u8(m_lastScanColor);
        static_cast<CSetOfObjectsPtr>(scene->getByName(aux_lastScan.ellipsoidsName))->setColor_u8(m_lastScanColor);
    }

    m_aux.insert({id,move(aux)});
}

void MSISViewer::initKeystrokes(const CConfigFile &cfg)
{
   loadKeystroke(cfg,"keystroke_MSIS_" + m_sensorLabel + "_toggle_scans"      ,keystroke("s", "toggle MSIS " + m_sensorLabel + " scan"           , bind(&MSISViewer::toggleScansVisualization, this)));
   loadKeystroke(cfg,"keystroke_MSIS_" + m_sensorLabel + "_toggle_ellipsoids", keystroke("e", "toggle MSIS " + m_sensorLabel + " scan ellipsoids", bind(&MSISViewer::toggleEllipsoidsVisualization, this)));
}

void MSISViewer::toggleEllipsoidsVisualization()
{
    // Use a variable to store the current state to avoid problem when adding a new scan
    // --> All the scans should share the same visibility, which is not the case if the previous scans were already toggle
    // (ie not visible) and a new scan arrive (visible by default)
    m_ellipsoidsVisible = !m_ellipsoidsVisible;
    for(const auto& pair : m_aux)
        setVisibility(pair.second.ellipsoidsName, m_ellipsoidsVisible);
}

void MSISViewer::toggleScansVisualization()
{
    m_scansVisible = !m_scansVisible;
    for(const auto& pair : m_aux)
        setVisibility(pair.second.pointCloudName,m_scansVisible);
}

void MSISViewer::getScanObjects(scanID id,
                                COpenGLScenePtr &scene,
                                MSISViewer_aux &aux,
                                CPointCloudPtr &pointCloud,
                                CSetOfObjectsPtr &ellipsoids,
                                CSetOfObjectsPtr &refFrameAxis)
{
    aux = m_aux[id];
    pointCloud = static_cast<CPointCloudPtr>(scene->getByName(aux.pointCloudName));
    ellipsoids = static_cast<CSetOfObjectsPtr>(scene->getByName(aux.ellipsoidsName));
    if(m_displayRefFrameAxis)
        refFrameAxis = static_cast<CSetOfObjectsPtr>(scene->getByName(aux.refFrameAxisName));

    pointCloud->clear();
    ellipsoids->clear();
}

void MSISViewer::renderScan(COpenGLScenePtr &scene,
                            const points_mat_t& localScanPts,
                            const mrpt::poses::CPose3D& refFramePose,
                            TNodeID nodeId)
{
    MRPT_START

    MSISViewer_aux aux;
    CPointCloudPtr pointCloud;
    CSetOfObjectsPtr ellipsoids, refFrameAxis;
    bool newScan = (m_aux.find(nodeId) != m_aux.end());
    // Check if first time scan and create corresponding opengl if it is the case
    newScan ? getScanObjects(nodeId, scene, aux, pointCloud, ellipsoids, refFrameAxis) :
              initNewScanObjects(nodeId, scene, aux, pointCloud, ellipsoids, refFrameAxis);


    // The scan has not been registered in a node ?
    points_mat_t pointsGlobalPoses = refFramePose.getHomogeneousMatrixVal()*localScanPts;

    // Extract the x, y and z values as rows of the previously compute matrix
    // Here points_mat_t MUST be RowMajor
    vector<float> x_vals, y_vals, z_vals;
    karst_slam::utils::pointsMatrixToCoordVectors(pointsGlobalPoses, localScanPts.cols(),
                                                  x_vals, y_vals, z_vals);


    // Set values in the cloud
    // ToDo : modify setAllPoints() to take reference (and avoid copy)
    pointCloud->setAllPoints(x_vals, y_vals, z_vals);

    // Now for the ellipsoids
    //if(m_displayEllipsoids)
    //{
    /*const eigenAlignedVector<cov_t>& covs = scan->m_cov;
    eigenAlignedVector<cov_t>::const_iterator covs_it = covs.begin();
    vector<float>::const_iterator x_it = x_vals.begin();
    vector<float>::const_iterator y_it = y_vals.begin();
    vector<float>::const_iterator z_it = z_vals.begin();
    mrpt::poses::CPose3D pointGlobalPose(refFramePose);
    mrpt::utils::TColor color = newScan ? m_lastScanColor : m_color;
    if(m_displayRefFrameAxis)
        refFrameAxis->setPose(pointGlobalPose);//scan->m_refFrameGlobalPoseAtCreation);

    for(; covs_it != covs.end();covs_it++, x_it++, y_it++, z_it++)
    {
        // Only change the translation component - avoir recomputing internally the rotation matrix
        pointGlobalPose.x() = *x_it;
        pointGlobalPose.y() = *y_it;
        pointGlobalPose.z() = *z_it;

        CEllipsoidPtr ellipsoid = CEllipsoid::Create();
        ellipsoid->setColor_u8(color);
        ellipsoid->setQuantiles(m_ellipsoidsQuantile);
        ellipsoid->setCovMatrix(*covs_it);
        ellipsoid->enableDrawSolid3D(false);
        ellipsoid->setPose(pointGlobalPose);
        ellipsoids->insert(ellipsoid);
    }*/

    //}

    MRPT_END
}
