#include "karst_slam/scanMerging/gui/scanMergingViewer_mrpt.h"
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/graphslam/misc/CWindowObserver.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/opengl/CAssimpModel.h>
#include <mrpt/opengl/CFrustum.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/CPointCloudColoured.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CCylinder.h>
#include <mrpt/opengl/CArrow.h>
#include <mrpt/opengl/CText.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/system.h>
#include <boost/math/distributions/beta.hpp>

using namespace std;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::opengl;
using namespace mrpt::math;
using namespace karst_slam::scanMerging;
using namespace karst_slam::gui;

const Eigen::Matrix3d permutationMatrix_toMRPTCylinderConvention = (Eigen::Matrix3d() << 0., 0., 1.,
                                                                    0., 1., 0.,
                                                                    1., 0., 0.)
                                                                       .finished();

simulationViewer_mrpt::simulationViewer_mrpt(const string &name,
                                             bool isSimulation) : MainViewer(name),
                                                                  m_isSimulation(isSimulation)
{
}

void simulationViewer_mrpt::setVerticalPointColor(const TColor &color)
{
    MRPT_START
    m_viewerParameters.verticalSonarPointsColor = color;

    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();
    CPointCloudPtr vertical_cloud = static_cast<CPointCloudPtr>(scene->getByName("verticalSonarPoints"));
    vertical_cloud->setColor_u8(color);

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void simulationViewer_mrpt::switchArcsModes()
{
    MRPT_START
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();
    CRenderizablePtr arcs = scene->getByName("arcs"),
                     modes_firstScan = scene->getByName("modes_firstScan"),
                     modes_secondScan = scene->getByName("modes_secondScan"),
                     arcsText = scene->getByName("arcsText"),
                     modesText = scene->getByName("modesText");
    arcs->setVisibility(!arcs->isVisible());
    modes_firstScan->setVisibility(!modes_firstScan->isVisible());
    modes_secondScan->setVisibility(!modes_secondScan->isVisible());

    // Display indexes on arcs (beams)
    if (arcs->isVisible() && modesText->isVisible())
    {
        modesText->setVisibility(false);
        arcsText->setVisibility(true);
    }
    // Display indexes on modes (local pdf maximum)
    else if (modes_firstScan->isVisible() && arcsText->isVisible())
    {
        modesText->setVisibility(true);
        arcsText->setVisibility(false);
    }
    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void simulationViewer_mrpt::toggleText()
{
    MRPT_START
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();
    CRenderizablePtr arcs = scene->getByName("arcs"),
                     arcsText = scene->getByName("arcsText"),
                     modesText = scene->getByName("modesText");

    if (arcs->isVisible())
        arcsText->setVisibility(!arcsText->isVisible());
    else
        modesText->setVisibility(!modesText->isVisible());

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void simulationViewer_mrpt::initKeystrokes(const CConfigFile &cfg)
{
    MRPT_START
    loadKeystroke(cfg, "keystroke_toggle_mean_GPSurface", keystroke("m", "Toggle mean of estimated surface", bind(&simulationViewer_mrpt::toggleVisualization, this, "mean_GP_surface")));
    loadKeystroke(cfg, "keystroke_toggle_lowerBound_GPSurface", keystroke("l", "Toggle lower bound of estimated surface", bind(&simulationViewer_mrpt::toggleVisualization, this, "lowerBound_GP_surface")));
    loadKeystroke(cfg, "keystroke_toggle_upperBound_GPSurface", keystroke("u", "Toggle upper bound of estimated surface", bind(&simulationViewer_mrpt::toggleVisualization, this, "upperBound_GP_surface")));
    loadKeystroke(cfg, "keystroke_toggle_cylinder", keystroke("p", "Toggle prior cylinder", bind(&simulationViewer_mrpt::toggleVisualization, this, "priorCylinders")));
    loadKeystroke(cfg, "keystroke_toggle_pca_cylinder", keystroke("f", "Toggle pca cylinder", bind(&simulationViewer_mrpt::toggleVisualization, this, "priorCylinders_pca")));
    loadKeystroke(cfg, "keystroke_toggle_normals", keystroke("n", "Display estimated normals", bind(&simulationViewer_mrpt::toggleVisualization, this, "normals")));
    loadKeystroke(cfg, "keystroke_toggle_grid", keystroke("g", "Display Grid", bind(&simulationViewer_mrpt::toggleVisualization, this, "Grid")));
    loadKeystroke(cfg, "keystroke_switchArcsModes", keystroke("a", "Switch from/to arcs to/from modes", bind(&simulationViewer_mrpt::switchArcsModes, this)));
    loadKeystroke(cfg, "keystroke_toggleText", keystroke("o", "Toggle arc/mode idx display", bind(&simulationViewer_mrpt::toggleText, this)));
    loadKeystroke(cfg, "keystroke_exit", keystroke("x", "Exit", bind(&simulationViewer_mrpt::exit, this)));

    if (m_isSimulation)
    {
        loadKeystroke(cfg, "keystroke_model", keystroke("r", "Display mesh model", bind(&simulationViewer_mrpt::toggleVisualization, this, "envModel")));
        loadKeystroke(cfg, "keystroke_wireframe", keystroke("w", "Display wire model", bind(&simulationViewer_mrpt::toggleVisualization, this, "envModelWire")));
        loadKeystroke(cfg, "keystroke_toggle_trajectoryPoses", keystroke("t", "Toggle trajectory poses frames", bind(&simulationViewer_mrpt::toggleVisualization, this, "trajectoryPoseFrames")));
        loadKeystroke(cfg, "keystroke_toggle_verticalSonarPoints", keystroke("v", "Toggle Vertical Sonar points", bind(&simulationViewer_mrpt::toggleVisualization, this, "verticalSonarPoints")));
    }

    MRPT_END
}

void simulationViewer_mrpt::loadParameters_simulation(const CConfigFile &cfg)
{
    m_beamWidth = DEG2RAD(cfg.read_double("SonarHorizontal", "beamWidth", 35, true));
    m_displayOnlyCurrentScan = cfg.read_bool("General", "displayOnlyCurrentScan", false, true);
    m_stopAtEachRendering = cfg.read_bool("General", "stopAtEachRendering", true, true);
    m_displayUniformArcs = cfg.read_bool("General", "displayUniformArcs", true, true);
    m_displayNormalUncertainty = cfg.read_bool("General", "displayNormalUncertainty", true, true);
    m_displayNormalDebugPoints = cfg.read_bool("General", "displayNormalDebugPoints", true, true);
}

void simulationViewer_mrpt::loadParameters_dataSet(const CConfigFile &cfg)
{
    m_beamWidth = DEG2RAD(cfg.read_double("MSIS", "Sonar_Micron_verticalBeamWidth", 35, true));
    m_displayOnlyCurrentScan = cfg.read_bool("Display", "displayOnlyCurrentScan", false, true);
    m_stopAtEachRendering = false;
    m_displayUniformArcs = cfg.read_bool("Display", "displayUniformArcs", true, true);
    m_displayNormalUncertainty = cfg.read_bool("Display", "displayNormalUncertainty", true, true);
    m_displayNormalDebugPoints = cfg.read_bool("Display", "displayNormalDebugPoints", true, true);
}

void simulationViewer_mrpt::initScene(const CConfigFile &cfg)
{
    MRPT_START

    // Load the parameters related to display
    if (m_isSimulation)
        loadParameters_simulation(cfg);
    else
        loadParameters_dataSet(cfg);

    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();
    scene->getViewport("main")->setCustomBackgroundColor(m_viewerParameters.backgroundColor);

    // World frame
    // Axis of the world frame
    CSetOfObjectsPtr worldFrame = stock_objects::CornerXYZ(1.f);
    worldFrame->setName("worldFrame");
    scene->insert(worldFrame);

    // Grid
    //CGridPlaneXYPtr grid = CGridPlaneXY::Create(0.,10,-20.,20.,0.,0.5);
    //grid->setName("Grid");
    //scene->insert(grid);

    // Environment model (only in simulation)
    if (m_isSimulation)
    {
        //string envModelFile     = cfg.read_string("Simulation", "envModel","",true);
        string envModelWireFile = cfg.read_string("Simulation", "envModelWire", "", true);
        CAssimpModelPtr //envModel     = CAssimpModel::Create(),
            envModelWire = CAssimpModel::Create();
        //envModel->loadScene(envModelFile);
        //envModel->setName("envModel");
        envModelWire->setColor_u8(0., 0., 0.);
        envModelWire->loadScene(envModelWireFile);
        envModelWire->setName("envModelWire");
        //scene->insert(envModel);
        scene->insert(envModelWire);
    }

    // Trajectory
    CSetOfObjectsPtr trajectory_poses = CSetOfObjects::Create();
    trajectory_poses->setName("trajectoryPoseFrames");
    scene->insert(trajectory_poses);

    CSetOfObjectsPtr trajectory_poses_gt = CSetOfObjects::Create();
    trajectory_poses_gt->setName("trajectoryPoseFrames_gt");
    scene->insert(trajectory_poses_gt);

    // Sonar points (measured/estimated)
    CPointCloudPtr vertical_cloud = CPointCloud::Create();
    //horizontal_cloud = CPointCloud::Create();
    vertical_cloud->setName("verticalSonarPoints");
    vertical_cloud->setPointSize(m_viewerParameters.pointSize_sonar);
    vertical_cloud->setColor_u8(m_viewerParameters.verticalSonarPointsColor);

    // Horizontal point cloud with theta=0 is not really needed so commented out
    /*horizontal_cloud->setName("horizontalSonarPoints");
    horizontal_cloud->setPointSize(m_viewerParameters.pointSize_sonar);
    horizontal_cloud->setColor_u8(m_viewerParameters.horizontalSonarPointsColor);*/

    scene->insert(vertical_cloud);
    //scene->insert(horizontal_cloud);

    // Surface (Gaussian Process regression with mean and  +/- 3-sigma bounds)
    CPointCloudPtr mean_GP_surface = CPointCloud::Create(),
                   lowerBound_GP_surface = CPointCloud::Create(),
                   upperBound_GP_surface = CPointCloud::Create();
    mean_GP_surface->setName("mean_GP_surface");
    mean_GP_surface->setPointSize(m_viewerParameters.pointSize_surface);
    mean_GP_surface->setColor_u8(m_viewerParameters.meanGPsurfaceColor);
    lowerBound_GP_surface->setName("lowerBound_GP_surface");
    lowerBound_GP_surface->setColor_u8(m_viewerParameters.lowerBoundGPsurfaceColor);
    lowerBound_GP_surface->setPointSize(m_viewerParameters.pointSize_surface);
    upperBound_GP_surface->setName("upperBound_GP_surface");
    upperBound_GP_surface->setColor_u8(m_viewerParameters.upperBoundGPsurfaceColor);
    upperBound_GP_surface->setPointSize(m_viewerParameters.pointSize_surface);

    CSetOfObjectsPtr normals = CSetOfObjects::Create();
    normals->setName("normals");
    CPointCloudPtr normalPoints = CPointCloud::Create();
    normalPoints->setName("debug_normalPoints");
    normalPoints->setColor_u8(m_viewerParameters.normalPtsColor);
    normalPoints->setPointSize(m_viewerParameters.pointSize_sonar);
    CSetOfObjectsPtr boundsCones = CSetOfObjects::Create();
    boundsCones->setName("normalBoundCones");

    scene->insert(mean_GP_surface);
    scene->insert(lowerBound_GP_surface);
    scene->insert(upperBound_GP_surface);
    scene->insert(normals);
    scene->insert(normalPoints);
    scene->insert(boundsCones);

    CSetOfObjectsPtr modesText = CSetOfObjects::Create(),
                     arcsText = CSetOfObjects::Create();
    modesText->setName("modesText");
    arcsText->setName("arcsText");
    scene->insert(modesText);
    scene->insert(arcsText);

    CPointCloudPtr modes_firstScan = CPointCloud::Create(),
                   modes_secondScan = CPointCloud::Create();
    modes_firstScan->setName("modes_firstScan");
    modes_secondScan->setName("modes_secondScan");
    modes_firstScan->setColor_u8(m_viewerParameters.modesFirstScan);
    modes_secondScan->setColor_u8(m_viewerParameters.modesSecondScan);
    modes_firstScan->setPointSize(m_viewerParameters.pointSize_sonar);
    modes_secondScan->setPointSize(m_viewerParameters.pointSize_sonar);
    modes_firstScan->setVisibility(false);
    modes_secondScan->setVisibility(false);
    scene->insert(modes_firstScan);
    scene->insert(modes_secondScan);

    CPointCloudColouredPtr arcs = CPointCloudColoured::Create();
    arcs->setName("arcs");
    arcs->setPointSize(m_viewerParameters.arcPointSize);
    scene->insert(arcs);

    // Prior cylinder
    CSetOfObjectsPtr priorCylinders = CSetOfObjects::Create();
    priorCylinders->setName("priorCylinders");
    scene->insert(priorCylinders);

    // Prior cylinder just with PCA (debug info)
    CSetOfObjectsPtr priorCylinders_pca = CSetOfObjects::Create();
    priorCylinders_pca->setName("priorCylinders_pca");
    scene->insert(priorCylinders_pca);
    /*
    CSetOfObjectsPtr priorCylinder_pca_frame = stock_objects::CornerXYZ(1);
    priorCylinder_pca_frame->setName("priorCylinder_pca_frame");
    scene->insert(priorCylinder_pca_frame);
*/

    CSetOfObjectsPtr priorCylinder_frame = stock_objects::CornerXYZ(1);
    priorCylinder_frame->setName("priorCylinder_frame");
    scene->insert(priorCylinder_frame);

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void simulationViewer_mrpt::renderPointCloud(COpenGLScenePtr &scene,
                                             const string &cloudName,
                                             const vector<CPointPDFGaussian> &pts,
                                             const vector<int> &ptsToRender)
{
    CPointCloudPtr ptsCloud = static_cast<CPointCloudPtr>(scene->getByName(cloudName));
    if (m_displayOnlyCurrentScan)
        ptsCloud->clear();

    // Empty mask
    if (ptsToRender.empty())
    {
        for (const CPointPDFGaussian &pt : pts)
            ptsCloud->insertPoint(pt.mean.x(), pt.mean.y(), pt.mean.z());
    }
    else
    {
        // Only render subset of points
        for (int idx : ptsToRender)
        {
            const CPoint3D &pt = pts[idx].mean;
            ptsCloud->insertPoint(pt.x(), pt.y(), pt.z());
        }
    }
}

void simulationViewer_mrpt::renderPointCloud(COpenGLScenePtr &scene,
                                             const string &cloudName,
                                             const vector<CPoint3D> &pts)
{
    CPointCloudPtr ptsCloud = static_cast<CPointCloudPtr>(scene->getByName(cloudName));
    if (m_displayOnlyCurrentScan)
        ptsCloud->clear();

    for (const CPoint3D &pt : pts)
        ptsCloud->insertPoint(pt.x(), pt.y(), pt.z());
}

void simulationViewer_mrpt::insertTrajectoryFrame(const CPose3D &pose,
                                                  CSetOfObjectsPtr trajPosesFrames)
{
    CSetOfObjectsPtr frame = stock_objects::CornerXYZSimple(m_viewerParameters.cornerFrameSize, 2.0);
    frame->setPose(pose);
    trajPosesFrames->insert(frame);
}

void simulationViewer_mrpt::renderTrajectory_(COpenGLScenePtr &scene,
                                              const string &name,
                                              const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> &bodyPoses,
                                              int interval)
{
    if (!bodyPoses.empty())
    {
        CSetOfObjectsPtr trajPosesFrames = static_cast<CSetOfObjectsPtr>(scene->getByName(name));
        CSetOfObjectsPtr trajPosesFrames_gt = static_cast<CSetOfObjectsPtr>(scene->getByName(name + "_gt"));
        if (interval == 1)
        {
            for (const auto &p : bodyPoses)
                insertTrajectoryFrame(p.second.pose.mean, trajPosesFrames);
        }
        else
        {
            auto it = bodyPoses.cbegin();
            for (int i = 0; i < bodyPoses.size(); i += interval)
            {
                insertTrajectoryFrame(it->second.pose.mean, trajPosesFrames);
                it = std::next(it, interval);
            }
        }
    }
}

void simulationViewer_mrpt::renderTrajectory_(COpenGLScenePtr &scene,
                                              const string &name,
                                              const map<uint64_t, trajectoryPose<CPose3D>> &bodyPoses,
                                              int interval)
{
    if (!bodyPoses.empty())
    {
        CSetOfObjectsPtr trajPosesFrames = static_cast<CSetOfObjectsPtr>(scene->getByName(name));
        CSetOfObjectsPtr trajPosesFrames_gt = static_cast<CSetOfObjectsPtr>(scene->getByName(name + "_gt"));
        // Display a frame for every pose in the trajectory
        if (interval == 1)
        {
            for (const auto &p : bodyPoses)
                insertTrajectoryFrame(p.second.pose, trajPosesFrames);
        }
        // Display a frame only every interval
        else
        {
            auto it = bodyPoses.cbegin();
            for (int i = 0; i < bodyPoses.size(); i += interval)
            {
                insertTrajectoryFrame(it->second.pose, trajPosesFrames);
                it = std::next(it, interval);
            }
        }
    }
}

void simulationViewer_mrpt::renderSurface(COpenGLScenePtr &scene,
                                          const vector<horizontalSonarMeasure> &horizontalSonarMeasures,
                                          const CPose3D &horizontalSonarPoseOnRobot,
                                          const scanMergingResults &res)
{
    renderPointCloud(scene, "mean_GP_surface", res.surfaceGPData.meanPoints);
    renderPointCloud(scene, "lowerBound_GP_surface", res.surfaceGPData.lowBoundPoints);
    renderPointCloud(scene, "upperBound_GP_surface", res.surfaceGPData.highBoundPoints);
    renderNormals(scene, horizontalSonarMeasures, horizontalSonarPoseOnRobot, res);
    if (m_displayNormalDebugPoints)
        renderNormalPoints_debug(scene, res.surfaceGPData.pointsForNormals_debug);
}

void simulationViewer_mrpt::renderNormalPoints_debug(COpenGLScenePtr &scene,
                                                     const vector<TPoint3D> &normalPoints)
{
    // Render the four points used to compute the normal (see Paper "On-manifold ..." in /doc)
    CPointCloudPtr normalPointsCloud = static_cast<CPointCloudPtr>(scene->getByName("debug_normalPoints"));
    if (m_displayOnlyCurrentScan)
        normalPointsCloud->clear();

    for (const TPoint3D &pt : normalPoints)
        normalPointsCloud->insertPoint(pt.x, pt.y, pt.z);
}

void simulationViewer_mrpt::renderNormal(const horizontalSonarMeasure &curHorizontalSonarMeasure,
                                         const mrpt::poses::CPose3D &horizontalSonarGlobalPose,
                                         const distributionTheta &dist,
                                         const CPointPDFGaussian &curNormal,
                                         CSetOfObjectsPtr normalsArrows,
                                         CSetOfObjectsPtr boundsCones)
{
    // Global pose of the estimated point (mode)
    // TODO : class / function to manage the distribution !
    double theta_mode;
    if (dist.alpha == 1 && dist.beta == 1)
        theta_mode = 0.;
    else if (dist.alpha <= 1.)
        theta_mode = -0.5 * m_beamWidth;
    else if (dist.beta <= 1.)
        theta_mode = 0.5 * m_beamWidth;
    else theta_mode = 0.5 * m_beamWidth * (dist.alpha - dist.beta) / (dist.alpha + dist.beta - 2.);

    TPoint3D modeInScanFrame = TPoint3D(curHorizontalSonarMeasure.rho * cos(theta_mode) * cos(curHorizontalSonarMeasure.yaw),
                                        curHorizontalSonarMeasure.rho * cos(theta_mode) * sin(curHorizontalSonarMeasure.yaw),
                                        curHorizontalSonarMeasure.rho * sin(theta_mode)), 
                                modeInGlobalFrame;

    horizontalSonarGlobalPose.composePoint(modeInScanFrame, modeInGlobalFrame);

    const CPoint3D &curNormalMean = curNormal.mean;
    CArrowPtr normal = CArrow::Create(modeInGlobalFrame.x,
                                      modeInGlobalFrame.y,
                                      modeInGlobalFrame.z,
                                      modeInGlobalFrame.x + 0.5 * curNormalMean.x(),
                                      modeInGlobalFrame.y + 0.5 * curNormalMean.y(),
                                      modeInGlobalFrame.z + 0.5 * curNormalMean.z());

    // Represent the cone with +-3 sigma
    if (m_displayNormalUncertainty)
    {
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(curNormal.cov);
        Eigen::Matrix3d P = es.eigenvectors();
        Eigen::Vector3d D = es.eigenvalues();
        Eigen::Vector3d curNormalMean_diagBasis = P.transpose() * curNormalMean.m_coords;
        Eigen::Vector3d incr = Eigen::Vector3d::Zero();

        double angles[3];
        for (int i = 0; i < 3; i++)
        {
            if (D(i) > 0)
            {
                //cout << "i = " << i << endl;
                incr = Eigen::Vector3d::Zero();
                incr(i) = sqrt(D(i));
                Eigen::Vector3d curBounds = P * (curNormalMean_diagBasis + incr);
                curBounds = curBounds / curBounds.norm();

                double dot = curBounds(0) * curNormalMean.m_coords[0] +
                             curBounds(1) * curNormalMean.m_coords[1] +
                             curBounds(2) * curNormalMean.m_coords[2];
                angles[i] = acos(dot);
            }
        }

        CCylinderPtr uncertaintyCone = CCylinder::Create();
        uncertaintyCone->setHeight(0.5);
        uncertaintyCone->setRadii(0., 1.);
        uncertaintyCone->setScale(0.5 * tan(angles[2]), 0.5 * tan(angles[1]), 1.);
        uncertaintyCone->setColor_u8(127, 0, 127, 127);
        uncertaintyCone->setSlicesCount(20);
        uncertaintyCone->setStacksCount(20);
        uncertaintyCone->setHasBases(false, false);

        CPose3D cylinderPose_inMRPTconvention;
        modeInGlobalFrame.getAsVector(cylinderPose_inMRPTconvention.m_coords);
        P.col(0) = curNormalMean.getAsVector();
        cylinderPose_inMRPTconvention.setRotationMatrix(P * permutationMatrix_toMRPTCylinderConvention);

        uncertaintyCone->setPose(cylinderPose_inMRPTconvention);
        boundsCones->insert(uncertaintyCone);
    }

    normal->setColor_u8(m_viewerParameters.normalVectorColor);
    normal->setLargeRadius(0.05);
    normal->setSmallRadius(0.01);
    normalsArrows->insert(normal);
}

void simulationViewer_mrpt::renderNormals_(int arcIdx, 
                                           const vector<distributionTheta> &curDistributions,
                                           const vector<horizontalSonarMeasure> &horizontalSonarMeasures,
                                           const CPose3D &horizontalSonarPoseOnRobot,
                                           const scanMergingResults &res,
                                           CSetOfObjectsPtr normalsArrows,
                                           CSetOfObjectsPtr boundsCones)
{
    const horizontalSonarMeasure &curHorizontalSonarMeasure = horizontalSonarMeasures[arcIdx];
    if (curHorizontalSonarMeasure.scanIdx == 0)
    {
        mrpt::poses::CPose3D horizontalSonarGlobalPose = curHorizontalSonarMeasure.robotGlobalPosePdf.mean + horizontalSonarPoseOnRobot;

        int normalIdx = 0;
        for (const distributionTheta &dist : curDistributions)
        {
            const CPointPDFGaussian &curNormal = res.surfaceGPData.normals[normalIdx];
            renderNormal(curHorizontalSonarMeasure, horizontalSonarGlobalPose, dist, curNormal, normalsArrows, boundsCones);

            normalIdx++;
        }
    }
}

void simulationViewer_mrpt::renderNormals(COpenGLScenePtr &scene,
                                          const vector<horizontalSonarMeasure> &horizontalSonarMeasures, 
                                          const CPose3D &horizontalSonarPoseOnRobot,
                                          const scanMergingResults &res)
{
    CSetOfObjectsPtr normalsArrows = static_cast<CSetOfObjectsPtr>(scene->getByName("normals"));
    CSetOfObjectsPtr boundsCones = static_cast<CSetOfObjectsPtr>(scene->getByName("normalBoundCones"));

    // Clear previous normals
    if (m_displayOnlyCurrentScan)
    {
        normalsArrows->clear();
        boundsCones->clear();
    }

    if (!res.surfaceGPData.normals.empty())
    {   
        int arcIdx;
        // For each elevation angle distribution (can be more than one per beam)
        for (const auto &pairArcIdxDist : res.distribution)
        {
            arcIdx = pairArcIdxDist.first;
            const vector<distributionTheta> &curDistributions = pairArcIdxDist.second;
            renderNormals_(arcIdx, 
                           curDistributions, 
                           horizontalSonarMeasures, 
                           horizontalSonarPoseOnRobot, 
                           res, normalsArrows, boundsCones);
        }
    }
}

/* void simulationViewer_mrpt::renderNormals(COpenGLScenePtr &scene,
                                          const vector<horizontalSonarMeasure> &horizontalSonarMeasures, 
                                          const CPose3D &horizontalSonarPoseOnRobot,
                                          const scanMergingResults &res)
{
    double half_b = 0.5 * m_beamWidth;

    CSetOfObjectsPtr normalsArrows = static_cast<CSetOfObjectsPtr>(scene->getByName("normals"));
    CSetOfObjectsPtr boundsCones = static_cast<CSetOfObjectsPtr>(scene->getByName("normalBoundCones"));

    // Clear previous normals
    if (m_displayOnlyCurrentScan)
    {
        normalsArrows->clear();
        boundsCones->clear();
    }

    if (!res.surfaceGPData.normals.empty())
    {
        int normalIdx = 0, arcIdx;
        double scale = 0.5;
        TPoint3D modeInScanFrame, modeInGlobalFrame;

        // For each elevation angle distribution (can be more than one per beam)
        for (const auto &pairArcIdxDist : res.distribution)
        {
            arcIdx = pairArcIdxDist.first;
            const vector<distributionTheta> &curDistributions = pairArcIdxDist.second;

            const horizontalSonarMeasure &curHorizontalSonarMeasure = horizontalSonarMeasures[arcIdx];
            if (curHorizontalSonarMeasure.scanIdx == 0)
            {
                mrpt::poses::CPose3D horizontalSonarGlobalPose = curHorizontalSonarMeasure.robotGlobalPosePdf.mean + horizontalSonarPoseOnRobot;

                TPoint3D modeInScanFrame, modeInGlobalFrame;
                for (const distributionTheta &dist : curDistributions)
                {
                    const CPointPDFGaussian &curNormal = res.surfaceGPData.normals[normalIdx];

                    // Global pose of the estimated point (mode)
                    // TODO : class / function to manage the distribution !
                    double theta_mode;
                    if (dist.alpha == 1 && dist.beta == 1)
                        theta_mode = 0.;
                    else if (dist.alpha <= 1.)
                        theta_mode = -half_b;
                    else if (dist.beta <= 1.)
                        theta_mode = half_b;
                    else
                        theta_mode = half_b * (dist.alpha - dist.beta) / (dist.alpha + dist.beta - 2.);

                    modeInScanFrame = TPoint3D(curHorizontalSonarMeasure.rho * cos(theta_mode) * cos(curHorizontalSonarMeasure.yaw),
                                               curHorizontalSonarMeasure.rho * cos(theta_mode) * sin(curHorizontalSonarMeasure.yaw),
                                               curHorizontalSonarMeasure.rho * sin(theta_mode));

                    horizontalSonarGlobalPose.composePoint(modeInScanFrame, modeInGlobalFrame);

                    const CPoint3D &curNormalMean = curNormal.mean;
                    CArrowPtr normal = CArrow::Create(modeInGlobalFrame.x,
                                                      modeInGlobalFrame.y,
                                                      modeInGlobalFrame.z,
                                                      modeInGlobalFrame.x + scale * curNormalMean.x(),
                                                      modeInGlobalFrame.y + scale * curNormalMean.y(),
                                                      modeInGlobalFrame.z + scale * curNormalMean.z());

                    // Represent the cone with +-3 sigma
                    if (m_displayNormalUncertainty)
                    {
                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(curNormal.cov);
                        Eigen::Matrix3d P = es.eigenvectors();
                        Eigen::Vector3d D = es.eigenvalues();
                        Eigen::Vector3d curNormalMean_diagBasis = P.transpose() * curNormalMean.m_coords;
                        Eigen::Vector3d incr = Eigen::Vector3d::Zero();

                        double angles[3];
                        for (int i = 0; i < 3; i++)
                        {
                            if (D(i) > 0)
                            {
                                //cout << "i = " << i << endl;
                                incr = Eigen::Vector3d::Zero();
                                incr(i) = sqrt(D(i));
                                Eigen::Vector3d curBounds = P * (curNormalMean_diagBasis + incr);
                                curBounds = curBounds / curBounds.norm();

                                double dot = curBounds(0) * curNormalMean.m_coords[0] +
                                             curBounds(1) * curNormalMean.m_coords[1] +
                                             curBounds(2) * curNormalMean.m_coords[2];
                                angles[i] = acos(dot);
                            }
                        }

                        CCylinderPtr uncertaintyCone = CCylinder::Create();
                        uncertaintyCone->setHeight(scale);
                        uncertaintyCone->setRadii(0., 1.);
                        uncertaintyCone->setScale(scale * tan(angles[2]), scale * tan(angles[1]), 1.);
                        uncertaintyCone->setColor_u8(127, 0, 127, 127);
                        uncertaintyCone->setSlicesCount(20);
                        uncertaintyCone->setStacksCount(20);
                        uncertaintyCone->setHasBases(false, false);

                        CPose3D cylinderPose_inMRPTconvention;
                        modeInGlobalFrame.getAsVector(cylinderPose_inMRPTconvention.m_coords);
                        P.col(0) = curNormalMean.getAsVector();
                        cylinderPose_inMRPTconvention.setRotationMatrix(P * permutationMatrix_toMRPTCylinderConvention);

                        uncertaintyCone->setPose(cylinderPose_inMRPTconvention);
                        boundsCones->insert(uncertaintyCone);
                    }

                    normal->setColor_u8(m_viewerParameters.normalVectorColor);
                    normal->setLargeRadius(0.05);
                    normal->setSmallRadius(0.01);
                    normalsArrows->insert(normal);

                    normalIdx++;
                }
            }
        }
    }
}*/

void simulationViewer_mrpt::renderCylinderFrame(COpenGLScenePtr &scene,
                                                const ellipticCylinder &cylinder)
{
    CSetOfObjectsPtr frame = static_cast<CSetOfObjectsPtr>(scene->getByName("priorCylinder_frame"));
    frame->setPose(cylinder.getPose());
}

CCylinderPtr simulationViewer_mrpt::generatePriorCylinder(const mrpt::utils::TColor& color,
                                                          const string& name,
                                                          const ellipticCylinder& cylinder)
{
    CCylinderPtr priorCylinder = CCylinder::Create(1.f, 1.f);
    priorCylinder->setSlicesCount(100);
    priorCylinder->setStacksCount(100);
    priorCylinder->setName(name);
    priorCylinder->setColor_u8(color);
    priorCylinder->setColorA(0.25);
    priorCylinder->setHeight(cylinder.getLength());
    priorCylinder->setScale(cylinder.getHalfSmallAxis(),
                            cylinder.getHalfLongAxis(),
                            1.f);
    
    return priorCylinder;
}

void simulationViewer_mrpt::setPriorCylinderPose(CCylinderPtr priorCylinder,
                                                 const ellipticCylinder &cylinder,
                                                 const CPose3D &cylinderPose)
{   
    const Eigen::Matrix3d &cylinderBase = cylinderPose.getRotationMatrix();

    // In mrpt, cylinder main axis is z ! and the center is the center of the bottom base
    CPose3D cylinderPose_inMRPTconvention;
    cylinderPose_inMRPTconvention.m_coords = cylinder.bottomBaseCenter();
    cylinderPose_inMRPTconvention.setRotationMatrix(cylinderBase * permutationMatrix_toMRPTCylinderConvention);

    priorCylinder->setPose(cylinderPose_inMRPTconvention);
}

void simulationViewer_mrpt::renderPriorCylinder(COpenGLScenePtr &scene,
                                                const ellipticCylinder &cylinder,
                                                const string& cylinderObjName,
                                                const string& cylinderName)
{
    CSetOfObjectsPtr priorCylinders = static_cast<CSetOfObjectsPtr>(scene->getByName(cylinderObjName));
    if (m_displayOnlyCurrentScan)
        priorCylinders->clear();

    const CPose3D &cylinderPose = cylinder.getPose();
    CCylinderPtr priorCylinder = generatePriorCylinder(m_viewerParameters.priorCylinderColor, 
                                                       cylinderName,
                                                       cylinder);
    setPriorCylinderPose(priorCylinder, cylinder, cylinder.getPose());
    priorCylinders->insert(priorCylinder);
}

void simulationViewer_mrpt::renderPriorCylinders(COpenGLScenePtr &scene,
                                                 const ellipticCylinder &cylinder)
{
    renderPriorCylinder(scene, cylinder, "priorCylinders", "priorCylinder");
    renderPriorCylinder(scene, cylinder, "priorCylinders_pca", "priorCylinder_pca");
}

void simulationViewer_mrpt::renderArcs(COpenGLScenePtr &scene,
                                       const vector<vector<pointThetaSimu>> &horizontalSonarPoints_thetaVals,
                                       const vector<horizontalSonarMeasure> &localHorizontalSonarMeasures,
                                       const map<int, vector<distributionTheta>> &distributions,
                                       const vector<int> &ptToRender)
{
    CPointCloudColouredPtr arcs = static_cast<CPointCloudColouredPtr>(scene->getByName("arcs"));

    CSetOfObjectsPtr arcsText = static_cast<CSetOfObjectsPtr>(scene->getByName("arcsText")),
                     modesText = static_cast<CSetOfObjectsPtr>(scene->getByName("modesText"));
    CPointCloudPtr modes_firstScan = static_cast<CPointCloudPtr>(scene->getByName("modes_firstScan")),
                   modes_secondScan = static_cast<CPointCloudPtr>(scene->getByName("modes_secondScan"));

    if (m_displayOnlyCurrentScan)
    {
        arcs->clear();
        modes_firstScan->clear();
        modes_secondScan->clear();
        arcsText->clear();
        modesText->clear();
    }

    if (ptToRender.empty())
    {
        for (int idx = 0; idx < horizontalSonarPoints_thetaVals.size(); idx++)
            createArcToRender(arcs, idx, horizontalSonarPoints_thetaVals, distributions, localHorizontalSonarMeasures[idx].scanIdx, arcsText, modesText, modes_firstScan, modes_secondScan);
    }
    else
    {
        for (int idx : ptToRender)
            createArcToRender(arcs, idx, horizontalSonarPoints_thetaVals, distributions, localHorizontalSonarMeasures[idx].scanIdx, arcsText, modesText, modes_firstScan, modes_secondScan);
    }
}

void simulationViewer_mrpt::createArcToRender(CPointCloudColouredPtr arcs,
                                              int idx,
                                              const vector<vector<pointThetaSimu>> &horizontalSonarPoints_thetaVals,
                                              const map<int, vector<distributionTheta>> &distribution,
                                              int scanIdx,
                                              CSetOfObjectsPtr arcsText,
                                              CSetOfObjectsPtr modesText,
                                              CPointCloudPtr modes_firstScan,
                                              CPointCloudPtr modes_secondScan)
{
    bool isUniform = false;
    map<int, boost::math::beta_distribution<>> beta_distributions;
    if (distribution.find(idx) != distribution.end())
    {
        const vector<distributionTheta> &curPtDistributions = distribution.at(idx);
        for (const distributionTheta &dist : curPtDistributions)
        {
            beta_distributions[dist.theta_idx] = boost::math::beta_distribution<>(dist.alpha, dist.beta);
            if (dist.alpha == dist.beta)
                isUniform = true;
        }
    }
    // Add uniform distribution
    else
    {
        isUniform = true;
        beta_distributions[0] = boost::math::beta_distribution<>(1, 1);
    }

    if (!isUniform || m_displayUniformArcs)
    {
        const vector<pointThetaSimu> &arcPts = horizontalSonarPoints_thetaVals.at(idx);
        float r, g, b, x;
        vector<CPoint3D> max_per_dist_pt(beta_distributions.size());
        vector<double> max_per_dist_pdf(beta_distributions.size(), -1);

        for (vector<pointThetaSimu>::const_iterator it = arcPts.begin(); it != arcPts.end(); it++)
        {
            const CPoint3D &curPt = it->point;
            x = (it->theta + 0.5 * m_beamWidth) / m_beamWidth + 1e-4; // Add small epsilon to avoid overflow when the mode is -+0.5*b !;
            double max_pdf = -1, cur_pdf, cur_pdf_max_per_dist = -1;

            int distIdx = 0;
            for (const auto &pair : beta_distributions)
            {
                cur_pdf = boost::math::pdf(pair.second, x);
                if (cur_pdf > /*max_pdf*/ max_per_dist_pdf[distIdx])
                {
                    max_pdf = cur_pdf;
                    max_per_dist_pdf[distIdx] = cur_pdf;
                    max_per_dist_pt[distIdx] = curPt;
                }
                // For arcs, display the color corresponding to the max at x of all the distributions
                if (cur_pdf > cur_pdf_max_per_dist)
                    cur_pdf_max_per_dist = cur_pdf;

                distIdx++;
            }
            if (isUniform)
            {
                max_per_dist_pt[0] = arcPts[0.5 * arcPts.size()].point;
            }

            float val = atan(m_beamWidth * cur_pdf_max_per_dist) / M_PI_2;
            if (scanIdx == 0)
                mrpt::utils::jet2rgb(val, r, g, b);
            else
                mrpt::utils::hot2rgb(val, r, g, b);

            arcs->push_back(curPt.x(), curPt.y(), curPt.z(),
                            r, g, b);
        }

        CPointCloud *curScan = (scanIdx == 0) ? modes_firstScan.get() : modes_secondScan.get();

        for (const CPoint3D &pt : max_per_dist_pt)
        {
            //curScan->insertPoint(pt.x(), pt.y(), pt.z());

            //mrpt::opengl::CTextPtr idxText = mrpt::opengl::CText::Create(to_string(idx));
            //idxText->setPose(pt + mrpt::poses::CPoint3D(0.,0.,0.1));
            //modesText->insert(idxText);
        }

        // For debug, display the idx of the arc
        /*mrpt::opengl::CTextPtr idxText = mrpt::opengl::CText::Create(to_string(idx));
        idxText->setPose(horizontalSonarPoints_thetaVals[idx][0].point + mrpt::poses::CPoint3D(0.,0.,0.1));
        arcsText->insert(idxText);*/
    }
}

void simulationViewer_mrpt::render(const shared_ptr<dataForScanMerging> &data,
                                   const scanMergingResults &res)
{
    MRPT_START
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();

    // Draw the trajectory
    renderTrajectory(scene, "trajectoryPoseFrames", data->bodyPoses, 20 /*interval*/);

    // Draw the detected points from the sonar
    vector<int> ptsToRender; // = {58};
    renderPointCloud(scene, "verticalSonarPoints", data->verticalSonarPoints);

    renderArcs(scene,
               data->horizontalSonarPoints_thetaVals,
               data->horizontalSonarMeasures,
               res.distribution,
               ptsToRender);

    // Draw estimate surface
    renderSurface(scene,
                  data->horizontalSonarMeasures,
                  data->horizontalSonarPoseOnRobot,
                  res);

    // Draw the prior cylinder
    renderPriorCylinders(scene, data->priorCylinder);
    renderCylinderFrame(scene, data->priorCylinder);

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END

    if (m_stopAtEachRendering)
    {
        eventLoop();
        m_exit = false;
    }
}

void simulationViewer_mrpt::render_loopClosure(const dataForScanMerging_loopClosure &data,
                                               const scanMergingResults &res_firstScan,
                                               const scanMergingResults &res_loopClosureScan)
{
    MRPT_START
    COpenGLScenePtr &scene = m_win->get3DSceneAndLock();

    cout << "render first scan" << endl;
    if (data.firstScanGrp != nullptr)
        render(data.firstScanGrp, res_firstScan);
    cout << "render loop scan" << endl;
    if (data.loopClosureScanGrp != nullptr)
    {
        // Change some display colors
        setVerticalPointColor(TColor(255, 0, 255));

        render(data.loopClosureScanGrp, res_loopClosureScan);
    }
    // Also render body poses provided by dead-reckoning (ie no interpolation) between the scans composing the loop closure
    //renderTrajectory(scene, "trajectoryPoseFrames", data.bodyPoses);

    m_win->unlockAccess3DScene();
    m_win->forceRepaint();
    MRPT_END
}

void simulationViewer_mrpt::setCamera(double azimuth_deg, double elevation_deg, double zoom,
                                      float ptTo_x, float ptTo_y, float ptTo_z)
{
    m_win->setCameraAzimuthDeg(azimuth_deg);
    m_win->setCameraElevationDeg(elevation_deg);
    m_win->setCameraZoom(zoom);
    m_win->setCameraPointingToPoint(ptTo_x, ptTo_y, ptTo_z);
    m_win->forceRepaint();
}
