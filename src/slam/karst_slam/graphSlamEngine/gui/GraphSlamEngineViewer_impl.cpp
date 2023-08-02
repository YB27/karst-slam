#include "GraphSlamEngineViewer_impl.h"
#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/sensors/Sensor.h"
#include "karst_slam/maps/Octomap.h"
#include "karst_slam/graphSlamEngine/GraphSlamEngine.h"
#include "karst_slam/sensors/gui/SensorViewerFactory.h"
#include "karst_slam/obs/ObservationMSISBeam.h"
#include "karst_slam/gui/utils.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/obs/CSensoryFrame.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::graphslam;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::gui;
using namespace karst_slam::graph;
using namespace karst_slam::slam;
using namespace karst_slam::sensors;
using namespace karst_slam::scanMerging;

GraphSlamEngineViewer_impl::GraphSlamEngineViewer_impl(const string& name) :
    simulationViewer_mrpt(name, false) 
{}

void GraphSlamEngineViewer_impl::render(GraphSlamEngine *engine)
{
    MRPT_START
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();

    const shared_ptr<PoseGraph>& pg = engine->getPoseGraph();
    const pose_pdf_t& robotLastEstPose = engine->getCurrentRobotPose();
    // ToDo : Inserting point in CPointCloud recreate its octree from scratch each time !!!! (all points are marked as new)
    // Fix it !
    renderOdometryOnlyPoses(scene, engine->getLastOdometryOnlyPose(), engine->getOdometryOnlyPoses().size());
    renderLastObservations(scene,
                          engine->getLastSensoryFramePtr()/*observations*/,
                          engine->getLastObservationPtr()/*observation*/,
                          engine->getLastGeneratedObservationPtr(),
                          robotLastEstPose);
    renderRobotEstPose(scene, robotLastEstPose);

    // Could be better by only redrawing parts being updated ? (in general when only a new node is inserted)
    if(engine->graphUpdated())
    {
        renderPoseGraph(scene, pg);
        //renderOdometryOnlyPoses(scene, engine->getLastOdometryOnlyPose()/* engine.getOdometryOnlyPoses()*/);
        //renderGTPoses(scene, m_enginePtr->getGTPoses());
        //renderLastObservations(scene, engine->getLastSensoryFramePtr()/*observations*/, engine->getLastObservationPtr()/*observation*/, robotLastEstPose);
        //renderRobotEstPose(scene, robotLastEstPose);
        //renderMap(scene,engine->getMap());
        renderScans(scene, engine->getScansPerNode(), pg);
    }

    // Display scan matching (surface and horizontal elevation angles)
    renderScanEstimatedData(scene,
                            engine->getERD()->getHorizontalSonarPose(),
                            engine->getERD()->getLastHorizontalSonarPoints_thetaVals(),
                            engine->getERD()->getLastHorizontalSonarMeasures(),
                            engine->getERD()->getTrainingData(),
                            engine->getERD()->getLastEllipticCylinder(),
                            engine->getERD()->getLastScanMergingResults());

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();

    MRPT_END
}

// TODO : don't use the generic initSensorViewers because it expects ObservationPtr (and we dont use it with MSIS)
// Leave the genericity for futur ....
void GraphSlamEngineViewer_impl::initMSISViewers(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    m_verticalSonarViewer   = make_shared<MSISViewer>("Sonar_Seaking", this);
    m_horizontalSonarViewer = make_shared<MSISViewer>("Sonar_Micron", this);
    m_verticalSonarViewer->init(scene, cfg);
    m_horizontalSonarViewer->init(scene, cfg);

    m_displayVerticalSonarMaxLimit = cfg.read_bool("Display", "displayVerticalSonarMaxLimit", true, true);

    // Add the keystroke from the sensor viewer
    addKeystrokes(m_verticalSonarViewer->getKeystrokes());
    addKeystrokes(m_horizontalSonarViewer->getKeystrokes());

    //createMiscObjects(scene, "Sonar_Micron", "MSIS_primary", cfg);
    //createMiscObjects(scene, "Sonar_Seaking", "MSIS_secondary", cfg);
}

void GraphSlamEngineViewer_impl::createMiscObjects(COpenGLScenePtr& scene,
                                                   const string &sensorName,
                                                   const string &sensorType,
                                                   const CConfigFile& cfg)
{
    if(sensorType == "MSIS_primary" || sensorType == "MSIS_secondary")
        createMiscObjects_MSIS(scene, sensorName, cfg);
}

void GraphSlamEngineViewer_impl::createMiscObjects_MSIS(COpenGLScenePtr& scene,
                                                        const string &sensorName,
                                                        const CConfigFile& cfg)
{
    // ToDo : Something wrong here as I have to copy code ... Should be in MSISViewer somehow but how to keep genericity as in initSensorViewers
    // while also allowing MSISViewer to display ObservationMSISBeam ?

    MRPT_START
    // Create a rectangular truncated pyramid representing the beam (with the right max range and width)
    // ToDo : Make a static version of MSIS::loadParams to return the param struct
    const string section_sensor = "MSIS";
    int maxRange         = cfg.read_float(section_sensor, sensorName + "_maxRange",20.f,true);
    float horizontal_fov = cfg.read_float(section_sensor, sensorName + "_horizontalBeamWidth",1.f ,true);
    float vertical_fov   = cfg.read_float(section_sensor, sensorName + "_verticalBeamWidth"  ,1.f ,true);
    CFrustumPtr beamField = CFrustum::Create(0.001f, (float)maxRange, horizontal_fov, vertical_fov, 1.5f, true, true);

    // Take the same color as the corresponding MSIS cloud points
    const string section_display = "Display_" + sensorName;
    unsigned char pointColor_r = cfg.read_int(section_display, "MSISScan_pointColor_r", 255,false);
    unsigned char pointColor_g = cfg.read_int(section_display, "MSISScan_pointColor_g", 0,false);
    unsigned char pointColor_b = cfg.read_int(section_display, "MSISScan_pointColor_b", 0,false);
    beamField->setColor(pointColor_r, pointColor_g, pointColor_b, 0.25 /* alpha */);
    beamField->setName(sensorName + "_fov");
    scene->insert(beamField);
    MRPT_END
}

void GraphSlamEngineViewer_impl::initScene(const CConfigFile& cfg)
{
    simulationViewer_mrpt::initScene(cfg);

    MRPT_START
    COpenGLScenePtr& scene = m_win->get3DSceneAndLock();

    // Init map display
    initMapDisplay(scene, cfg);

    // Init odoOnlyPoses display
    initOdometryOnlyDisplay(scene, cfg);

    // Init ground truth display
    //initGroundTruthDisplay(scene, cfg);

    // Init Pose Graph display
    initPoseGraphDisplay(scene, cfg);

    // Init robot model
    //initRobotModel(scene, cfg);

    // Init misc
    initFixedObjects(scene, cfg);

    // Init all sensors display
    initMSISViewers(scene, cfg);

    if(m_displayVerticalSonarMaxLimit)
    {
        CPointCloudPtr offRangesCloud = CPointCloud::Create();
        offRangesCloud->setName("offRangesCloud");
        offRangesCloud->setColor(51./255.,232./255.,1.);
        offRangesCloud->setPointSize(3.f);
        scene->insert(offRangesCloud);
    }
    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void GraphSlamEngineViewer_impl::initMapDisplay(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    MRPT_START
    COctoMapVoxelsPtr map_obj = COctoMapVoxels::Create();
    COctoMapVoxels::visualization_mode_t vizu_mode = (COctoMapVoxels::visualization_mode_t) cfg.read_int(m_section,
                                                                                                         "Octomap_visualization_mode",
                                                                                                         COctoMapVoxels::COLOR_FROM_OCCUPANCY,
                                                                                                         false);
    bool enableLights = cfg.read_bool(m_section, "Octomap_enableLights", true, false);
    bool showGridLines = cfg.read_bool(m_section, "Octomap_showGridLines", false, false);
    float gridLinesWidth = cfg.read_float(m_section, "Octomap_gridLinesWidth", 1.f, false);
    bool showFreeVoxels = cfg.read_bool(m_section, "Octomap_showFreeVoxels", false, false);
    bool showVoxelsAsPoints = cfg.read_bool(m_section, "Octomap_showVoxelsAsPoints", false, false);

    map_obj->setVisualizationMode(vizu_mode);
    map_obj->setGridLinesWidth(gridLinesWidth);
    map_obj->showGridLines(showGridLines);
    map_obj->enableLights(enableLights);
    map_obj->showVoxels(VOXEL_SET_FREESPACE, showFreeVoxels);
    map_obj->showVoxelsAsPoints(showVoxelsAsPoints);

    map_obj->setName("map");
    scene->insert(map_obj);
    MRPT_END
}

void GraphSlamEngineViewer_impl::initOdometryOnlyDisplay(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    MRPT_START
    CPointCloudPtr odometry_poses_cloud = CPointCloud::Create();

    float pointSize = cfg.read_float(m_section, "OdometryOnly_pointSize", 1.f, false);
    unsigned char pointColor_r = cfg.read_int(m_section, "OdometryOnly_pointColor_r", 0,false);
    unsigned char pointColor_g = cfg.read_int(m_section, "OdometryOnly_pointColor_g", 0,false);
    unsigned char pointColor_b = cfg.read_int(m_section, "OdometryOnly_pointColor_b", 255,false);

    odometry_poses_cloud->setPointSize(pointSize);
    odometry_poses_cloud->enablePointSmooth();
    TColor color(pointColor_r, pointColor_g, pointColor_b);
    odometry_poses_cloud->setColor_u8(color);

    odometry_poses_cloud->setName("odometry_poses_cloud");
    scene->insert(odometry_poses_cloud);

    // Robot model at last pose
    CRenderizablePtr robModel =  initRobotModel(cfg);
    robModel->setName("robotModel_odo");
    robModel->setColor_u8(color);
    robModel->setPose(::CPose3D());
    scene->insert(robModel);

    m_prev_nOdo = 0;

    MRPT_END
}

void GraphSlamEngineViewer_impl::initGroundTruthDisplay(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    MRPT_START
    CPointCloudPtr GT_cloud = CPointCloud::Create();
    float pointSize = cfg.read_float(m_section, "GT_pointSize", 1.f, false);
    unsigned char pointColor_r = cfg.read_int(m_section, "GT_pointColor_r", 0,false);
    unsigned char pointColor_g = cfg.read_int(m_section, "GT_pointColor_g", 255,false);
    unsigned char pointColor_b = cfg.read_int(m_section, "GT_pointColor_b", 0,false);

    GT_cloud->setPointSize(pointSize);
    GT_cloud->enablePointSmooth();
    GT_cloud->enableColorFromX(false);
    GT_cloud->enableColorFromY(false);
    GT_cloud->enableColorFromZ(false);
    TColor color(pointColor_r, pointColor_g, pointColor_b);
    GT_cloud->setColor_u8(color);
    GT_cloud->setName("GT_cloud");

    scene->insert(GT_cloud);

    // Robot model at last pose
    CRenderizablePtr robModel =  initRobotModel(cfg);
    robModel->setName("robotModel_gt");
    robModel->setColor_u8(color);
    robModel->setPose(CPose3D());
    scene->insert(robModel);

    MRPT_END
}

void GraphSlamEngineViewer_impl::initPoseGraphDisplay(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    MRPT_START
    // Estimated trajectory
    CSetOfObjectsPtr estimated_nodePoses = CSetOfObjects::Create();
    estimated_nodePoses->setName("Estimated_nodePoses");
    scene->insert(estimated_nodePoses);

    CSetOfLinesPtr estimated_traj = CSetOfLines::Create();

    unsigned char traj_color_r = cfg.read_int(m_section, "EstimatedTraj_color_r", 127, false);
    unsigned char traj_color_g = cfg.read_int(m_section, "EstimatedTraj_color_g", 0, false);
    unsigned char traj_color_b = cfg.read_int(m_section, "EstimatedTraj_color_b", 127, false);
    float lineWidth = cfg.read_float(m_section, "EstimatedTraj_lineWidth", 1.5f, false);

    TColor color(traj_color_r, traj_color_g, traj_color_b);
    estimated_traj->setColor_u8(color);
    estimated_traj->setLineWidth(lineWidth);
    estimated_traj->setName("Estimated_traj");
    estimated_traj->appendLine(0, 0, 0,
                               0, 0, 0);

    scene->insert(estimated_traj);

    // Robot model at last Node
    CRenderizablePtr robModel = initRobotModel(cfg);
    robModel->setName("robotModel_est");
    robModel->setColor_u8(color);
    robModel->setPose(CPose3D());
    scene->insert(robModel);

    // Edges related visuals
    // ... cf CVisualizer
    MRPT_END
}

CRenderizablePtr GraphSlamEngineViewer_impl::initRobotModel(const CConfigFile& cfg)
{
    // Here load a .obj (or other 3D model format of the robot model)
    // By default, use axis frame
    string robotModelFile = cfg.read_string(m_section, "robotModel_file", "", true);
    CRenderizablePtr robModel;
    if(!robotModelFile.empty())
    {
        CAssimpModelPtr robModel_fromFile = CAssimpModel::Create();
        robModel_fromFile->loadScene(robotModelFile);
        robModel = robModel_fromFile;
    }
    else
        robModel = stock_objects::CornerXYZSimple(5);

    return robModel;
}

void GraphSlamEngineViewer_impl::initFixedObjects(COpenGLScenePtr& scene, const CConfigFile& cfg)
{
    MRPT_START

    // Axis of the world frame
    CSetOfObjectsPtr worldFrame = stock_objects::CornerXYZSimple(5);
    worldFrame->setName("worldFrame");
    scene->insert(worldFrame);

    MRPT_END
}

void GraphSlamEngineViewer_impl::initKeystrokes(const CConfigFile& cfg)
{
    simulationViewer_mrpt::initKeystrokes(cfg);

    MRPT_START
    loadKeystroke(cfg,"keystroke_toggle_odometryOnly_traj",keystroke("o"     ,"Toggle Odometry only trajectory visualization" , bind(&GraphSlamEngineViewer_impl::toggleVisualization, this, "odometry_poses_cloud")));
    loadKeystroke(cfg,"keystroke_toggle_grid"             ,keystroke("g"     ,"Toggle Grid visualization"                     , bind(&GraphSlamEngineViewer_impl::toggleVisualization, this, "grid")));
    loadKeystroke(cfg,"keystroke_toggle_estimated_traj"   ,keystroke("t"     ,"Toggle Estimated trajectory visualization"     , bind(&GraphSlamEngineViewer_impl::toggleVisualization, this, "Estimated_traj")));
    loadKeystroke(cfg,"keystroke_toggle_map"              ,keystroke("m"     ,"Toggle Map visualization"                      , bind(&GraphSlamEngineViewer_impl::toggleVisualization, this, "map")));
    loadKeystroke(cfg,"keystroke_exit"                    ,keystroke("Ctrl+c","Exit program"                                  , bind(&GraphSlamEngineViewer_impl::exit               , this )));
    loadKeystroke(cfg,"keystroke_resume"                  ,keystroke("r"     ,"Resume program"                                , bind(&GraphSlamEngineViewer_impl::resume             , this )));
    MRPT_END
}

void GraphSlamEngineViewer_impl::renderPoseGraph(COpenGLScenePtr& scene,
                                                 const shared_ptr<PoseGraph>& pose_graph)
{
    MRPT_START
    // ToDo : only modify update part of the graph
    CSetOfLinesPtr estimated_traj = static_cast<CSetOfLinesPtr>(scene->getByName("Estimated_traj"));
    estimated_traj->clear(); // To be remove ...
    // dummy way so that I can use appendLineStrip afterwards.
    estimated_traj->appendLine(
                /* 1st */ 0, 0, 0,
                /* 2nd */ 0, 0, 0);

    map<TNodeID, pose_t> nodesPose = pose_graph->getNodes();
    PoseGraph::edges_map_t edges = pose_graph->getEdges();

    // append line for each node in the set
    for(const auto& pair : edges)
    {
        const pose_t& node_from = nodesPose[pair.first.first];
        const pose_t& node_to   = nodesPose[pair.first.second];
        estimated_traj->appendLine(
                    node_from.x(),node_from.y(),node_from.z(),
                    node_to.x(),node_to.y(),node_to.z());
    }

    // Node poses
    // TODO : optimize by just setting adding new frame for new node and only modified node poses
    // (have a vector of lastly modified node in pose_graph?)
    CSetOfObjectsPtr estimated_nodePoses = static_cast<CSetOfObjectsPtr>(scene->getByName("Estimated_nodePoses"));
    estimated_nodePoses->clear();
    string nodeName;
    for(const auto& pair : nodesPose)
    {   
        nodeName = "nodeFrame_" + to_string(pair.first);
        CRenderizablePtr frame = estimated_nodePoses->getByName(nodeName);
        
        // First time displaying this node ? 
        if(frame == NULL)
        {
            frame = stock_objects::CornerXYZSimple(1);
            frame->setName("nodeName");
            estimated_nodePoses->insert(frame);
        }
        frame->setPose(pair.second);
    }

    MRPT_END
}

void GraphSlamEngineViewer_impl::renderPointCloudPoses(COpenGLScenePtr& scene, const vector<pose_t>& poses, const string& objName)
{
    MRPT_START
    CPointCloudPtr point_cloud = static_cast<CPointCloudPtr>(scene->getByName(objName));
    for(const auto& p : poses)
        point_cloud->insertPoint(p.x(), p.y(), p.z());

    MRPT_END
}

void GraphSlamEngineViewer_impl::renderOdometryOnlyPoses(COpenGLScenePtr& scene,
                                                         const pose_t& lastOdoPose,
                                                         size_t nOdo)
{
    MRPT_START
    if(nOdo > m_prev_nOdo)
    {
        CPointCloudPtr point_cloud = static_cast<CPointCloudPtr>(scene->getByName("odometry_poses_cloud"));
        point_cloud->insertPoint(lastOdoPose.x(), lastOdoPose.y(), lastOdoPose.z());
        CRenderizablePtr robModel = scene->getByName("robotModel_odo");
        robModel->setPose(CPose3D(lastOdoPose));
        m_prev_nOdo = nOdo;
    }
    MRPT_END
}

void GraphSlamEngineViewer_impl::renderGTPoses(COpenGLScenePtr& scene, const vector<pose_t>& gtPoses)
{
    MRPT_START
    renderPointCloudPoses(scene, gtPoses, "GT_cloud");
    CRenderizablePtr robModel = scene->getByName("robotModel_gt");
    robModel->setPose(CPose3D(gtPoses.back()));
    MRPT_END
}

void GraphSlamEngineViewer_impl::renderMap(COpenGLScenePtr& scene, const shared_ptr<OctoMap>& map)
{
    MRPT_START
    COctoMapVoxelsPtr octomap = static_cast<COctoMapVoxelsPtr>(scene->getByName("map"));
    octomap->setFromOctoMap(*map);
    MRPT_END
}

void GraphSlamEngineViewer_impl::renderScans(COpenGLScenePtr& scene,
                                             const node_to_obs_t& scans_per_node,
                                             const shared_ptr<PoseGraph>& poseGraph)
{
    for(const auto& pair: scans_per_node)
    {
        const CPose3D& refFramePose =  poseGraph->getNodePose(pair.first);
        m_verticalSonarViewer->renderScan(scene, 
                                          pair.second->verticalSonarPoints, 
                                          refFramePose,
                                          pair.first);
        m_horizontalSonarViewer->renderScan(scene, 
                                          pair.second->horizontalSonarPoints, 
                                          refFramePose,
                                          pair.first);
    }

    if(m_displayVerticalSonarMaxLimit)
    {
        // Stack the limit points
        points_mat_t pointsGlobalPoses;
        vector<float> x_vals, y_vals, z_vals;
        for(const auto& pair: scans_per_node)
        {
            const CPose3D& refFramePose =  poseGraph->getNodePose(pair.first);
            pointsGlobalPoses = refFramePose.getHomogeneousMatrixVal()*pair.second->offRangeVerticalSonarPoints;

            vector<float> cur_x, cur_y, cur_z;
            karst_slam::utils::pointsMatrixToCoordVectors(pointsGlobalPoses, pair.second->offRangeVerticalSonarPoints.cols(),
                                                          cur_x, cur_y, cur_z);
            x_vals.insert(x_vals.end(), 
                          make_move_iterator(cur_x.begin()), 
                          make_move_iterator(cur_x.end()));
            y_vals.insert(y_vals.end(), 
                          make_move_iterator(cur_y.begin()), 
                          make_move_iterator(cur_y.end()));
            z_vals.insert(z_vals.end(), 
                          make_move_iterator(cur_z.begin()), 
                          make_move_iterator(cur_z.end()));

        }
        
        CPointCloudPtr pointCloud = static_cast<CPointCloudPtr>(scene->getByName("offRangesCloud"));
        pointCloud->setAllPoints(x_vals, y_vals, z_vals);
    }
    
}

void GraphSlamEngineViewer_impl::renderLastObservations(COpenGLScenePtr& scene,
                                                        const mrpt::obs::CSensoryFramePtr& observations,
                                                        const mrpt::obs::CObservationPtr& observation,
                                                        const CSensoryFramePtr &generatedObservations,
                                                        const pose_pdf_t &currentRobotPose)
{
    MRPT_START
    if(observation)
        renderObservation(scene, observation,currentRobotPose);
    else
    {
        for(CSensoryFrame::const_iterator it_obs = observations->begin(); it_obs != observations->end();it_obs++)
            renderObservation(scene, *it_obs,currentRobotPose);
    }
    MRPT_END
}

void GraphSlamEngineViewer_impl::renderObservation(COpenGLScenePtr& scene,
                                                   const mrpt::obs::CObservationPtr& observation,
                                                   const pose_pdf_t &currentRobotPose)
{
    using namespace karst_slam::obs;
    if(IS_CLASS(observation,ObservationMSISBeam))
        renderMSIS_frustum(scene, static_cast<karst_slam::obs::ObservationMSISBeamPtr>(observation), currentRobotPose.getMeanVal());
    // else if(...)
}

void GraphSlamEngineViewer_impl::renderMSIS_frustum(COpenGLScenePtr& scene,
                                                    const karst_slam::obs::ObservationMSISBeamPtr& obs_beam,
                                                    const pose_t/*CPose3D*/ &currentRobotPose)
{
    CFrustumPtr msisBeam_fov = static_cast<CFrustumPtr>(scene->getByName(obs_beam->sensorLabel + "_fov"));

    // Could be taken once
    CPose3D poseOnRobot_ypr;
    obs_beam->getSensorPose(poseOnRobot_ypr);
    pose_t poseOnRobot(poseOnRobot_ypr);

    pose_t beamPoseInSensorFrame(CPose3D(0.,0.,0.,obs_beam->getAngle(),0.,0.));
    msisBeam_fov->setPose(CPose3D(currentRobotPose + poseOnRobot + beamPoseInSensorFrame));
}

void GraphSlamEngineViewer_impl::renderRobotEstPose(COpenGLScenePtr& scene,
                                                    const pose_pdf_t& lastRobotEstPose)
{
    CRenderizablePtr robModel = scene->getByName("robotModel_est");
    robModel->setPose(lastRobotEstPose.mean);

    // ToDo : draw incertitude ellipsoids
}

void GraphSlamEngineViewer_impl::renderScanEstimatedData(COpenGLScenePtr& scene,
                                                         const CPose3D& horizontalSonarPoseOnRobot,
                                                         const vector<vector<pointThetaSimu>>& horizontalSonarPoints_thetaVals,
                                                         const vector<horizontalSonarMeasure>& horizontalSonarMeasures,
                                                         const surfaceTrainingData& td,
                                                         const ellipticCylinder& priorCylinder,
                                                         const scanMergingResults& scanRes)
{
    renderArcs(scene,
               horizontalSonarPoints_thetaVals,
               horizontalSonarMeasures,
               scanRes.distribution);
    
    ///////////////////// DEBUG
    // Render the vertical sonar point in trainingData
    /*vector<CPoint3D> pts;
    const vector<surfaceDatum>& data = td.getData();
    pts.reserve(data.size());
    for(const surfaceDatum& d: data)
        pts.push_back(CPoint3D(d.pt));
    //cout << "[GraphSlamEngineViewer_impl::renderScanEstimatedData] verticalSonarPts : " << endl;
    //for(const CPoint3D& pt : pts)
    //    cout << pt << endl; 
    renderPointCloud(scene, "verticalSonarPoints", pts);*/
    /////////////////////

    // Draw estimate surface
    renderSurface(scene, horizontalSonarMeasures, horizontalSonarPoseOnRobot, scanRes);

    // Draw the prior cylinder
    renderPriorCylinders(scene, priorCylinder);
    renderCylinderFrame(scene, priorCylinder);
}
