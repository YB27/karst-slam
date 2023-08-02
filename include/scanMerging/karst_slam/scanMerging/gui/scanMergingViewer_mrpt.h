#ifndef DEF_SIMULATION_VIEWER_MRPT_H
#define DEF_SIMULATION_VIEWER_MRPT_H

#include "karst_slam/scanMerging/dataForScanMerging.h"
#include "karst_slam/gui/MainViewer.h"

// Fwrd dcl
namespace mrpt
{
    namespace poses
    {
        class CPoint3D;
        class CPointPDFGaussian;
        class CPose3DPDFGaussian;
    }

    namespace opengl
    {
        class CPointCloudPtr;
        class CPointCloudColouredPtr;
        class CSetOfLinesPtr;
        class CSetOfObjectsPtr;
        class CCylinderPtr;
    }

    namespace utils{class CConfigFile;}
    namespace gui{class CDisplayWindow3D;}
    namespace graphslam{class CWindowObserver;}
}

/**
 * This class is used to render the 3D simulation for scan merging.
 * It is based on mrpt 3D opengl render class.
 * Previously, used libigl viewer but it is very limited for customization outside of mesh rendering.
 */
namespace karst_slam{namespace gui{
class simulationViewer_mrpt : public MainViewer
{
public:

    /** Structure regrouping the parameters used for display (mainly sizes and color) */
    struct viewerParameters
    {
        float pointSize_surface = 5.f;
        float pointSize_sonar   = 8.f;
        float cornerFrameSize   = 0.1f;
        float arcPointSize      = 8.f;
        mrpt::utils::TColor verticalSonarPointsColor   = mrpt::utils::TColor(255,0,0);
        mrpt::utils::TColor horizontalSonarPointsColor = mrpt::utils::TColor(255,127,0);
        mrpt::utils::TColor meanGPsurfaceColor         = mrpt::utils::TColor(0,255,0,80);
        mrpt::utils::TColor lowerBoundGPsurfaceColor   = mrpt::utils::TColor(0,0,127);
        mrpt::utils::TColor upperBoundGPsurfaceColor   = mrpt::utils::TColor(0,0,255);
        mrpt::utils::TColor normalPtsColor             = mrpt::utils::TColor(0,0,0);
        mrpt::utils::TColor normalVectorColor          = mrpt::utils::TColor(0,127,127);
        mrpt::utils::TColor modesFirstScan             = mrpt::utils::TColor(255,0,0);
        mrpt::utils::TColor modesSecondScan            = mrpt::utils::TColor(0,0,255);
        mrpt::utils::TColor priorCylinderPCAColor      = mrpt::utils::TColor(255,165,51);
        mrpt::utils::TColor priorCylinderColor         = mrpt::utils::TColor(102,0,255);
        mrpt::utils::TColorf backgroundColor           = mrpt::utils::TColorf(0.6f,0.6f,0.6f);
    };

    /** Constructor 
     * 
     * @param name Name of the window
     * @param isSimulation If true, try to load the given 3D environment model used for simulation
    */
    simulationViewer_mrpt(const std::string& name, bool isSimulation);
    virtual ~simulationViewer_mrpt() = default;

    /**
     * Set the camera position in the 3D view
     * 
     * @param azimuth_deg Azimuth angle in degree
     * @param elevation_deg Elevation angle in degree
     * @param zoom Distance from the pointed point
     * @param pointedTo_x Point pointed by the camera (x coord)
     * @param pointedTo_y Point pointed by the camera (y coord)
     * @param pointedTo_z Point pointed by the camera (z coord)
     */
    void setCamera(const double azimuth_deg, const double elevation_deg, const double zoom,
                   const float pointTo_x, const float pointTo_y, const float pointTo_z);

    // TODO : Impl functions to update all display parameters
    void setVerticalPointColor(const mrpt::utils::TColor& color);

    /**
     * Render the opengl scene
     * 
     * @param data Contains all the simulation data to be displayed
     * @param res Contains results from the scan merging (using vertical sonar to estimate elevation angle for the horizontal sonar, 
     * see  Breux, Yohan, and Lionel Lapierre. "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
     * for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.)  
     */
    void render(const std::shared_ptr<scanMerging::dataForScanMerging>& data,
                const scanMerging::scanMergingResults& res);            
    /**
     * Render the opengl scene for loop closure matching
     * 
     * @param data Contains all the simulation data to be displayed
     * @param res_firstScan Contains results from the scan merging (fusion of vertical and horizontal sonar) for the first full scan
     * @param res_loopClosureScan Contains results from the scan merging (fusion of vertical and horizontal sonar) for the second full scan after loop closure
     */ 
    void render_loopClosure(const scanMerging::dataForScanMerging_loopClosure& data,
                            const scanMerging::scanMergingResults& res_firstScan,
                            const scanMerging::scanMergingResults& res_loopClosureScan);

    /** Switch the arcs (sonar beams) representation from full arc to/from only mode display (one point) */
    void switchArcsModes();

    /** Hide/display the index of each arcs (beam) */
    void toggleText();

protected:
    /**
     * Render a point cloud
     * 
     * @param scene Opengl scene
     * @param cloudName Name of the point cloud
     * @param pts Points to be addedqsssss in the point cloud
     * @param ptsToRender (Mask) vector of index of points to render
     */
    void renderPointCloud(mrpt::opengl::COpenGLScenePtr& scene,
                          const std::string& cloudName,
                          const std::vector<mrpt::poses::CPointPDFGaussian>& pts,
                          const std::vector<int>& ptsToRender = std::vector<int>());

    /**
     * \overload
     */
    void renderPointCloud(mrpt::opengl::COpenGLScenePtr& scene,
                          const std::string& cloudName,
                          const std::vector<mrpt::poses::CPoint3D>& pts);

    
    void renderTrajectory_(mrpt::opengl::COpenGLScenePtr& scene,
                           const std::string& name,
                           const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& bodyPoses,
                           int interval);
    void renderTrajectory_(mrpt::opengl::COpenGLScenePtr& scene,
                           const std::string& name,
                           const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3D>>& bodyPoses,
                           int interval);
    /**
     * Render the robot trajectory
     * 
     * @param scene Opengl scene
     * @param bodyPoses vector of robot body poses
     */
    template<class POSE>
    void renderTrajectory(mrpt::opengl::COpenGLScenePtr& scene,
                          const std::string& name,
                          const std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<POSE>>& bodyPoses,
                          int interval = 1)
    {
        renderTrajectory_(scene, name, bodyPoses, interval);
    }
    
    void insertTrajectoryFrame(const mrpt::poses::CPose3D& pose,
                               mrpt::opengl::CSetOfObjectsPtr trajPosesFrames);

    /**
     * Render the estimated surface with gaussian process
     * 
     * @param scene Opengl scene
     * @param surface Probabilistic surface to be displayed
     */
    void renderSurface(mrpt::opengl::COpenGLScenePtr& scene,
                       const std::vector<scanMerging::horizontalSonarMeasure>& horizontalSonarMeasures,
                       const mrpt::poses::CPose3D& horizontalSonarPoseOnRobot,
                       const scanMerging::scanMergingResults& res);

    /**
     * Render the prior surface estimation 
     * 
     * @param scene Opengl scene
     * @param priorCylinder Prior elliptic cylinder
     * @param cylinderObjName Opengl object containing PCA and optimized cylinder
     * @param cylinderName name of the prior cylinder
     */ 
    void renderPriorCylinder(mrpt::opengl::COpenGLScenePtr &scene,
                             const scanMerging::ellipticCylinder &cylinder,
                             const std::string& cylinderObjName,
                             const std::string& cylinderName);

    /**
     * Render the prior surface estimation (an elliptic cylinder, both raw Robust PCA and optimized cylinder)
     * 
     * @param scene Opengl scene
     * @param priorCylinder Prior elliptic cylinder
     */
    void renderPriorCylinders(mrpt::opengl::COpenGLScenePtr& scene,
                              const scanMerging::ellipticCylinder& priorCylinder);

    /**
     * Render the frame axis of an elliptic cylinder (mainly used for PCA cylinder, debug)
     *
     * @param scene Opengl scene
     * @param cylinder prior elliptic cylinder
     */
    void renderCylinderFrame(mrpt::opengl::COpenGLScenePtr &scene,
                             const scanMerging::ellipticCylinder &cylinder);

    /**
     * Generate one arc
     * 
     * @param arcs Point clouds used to represen the arcs (sonar beams) 
     * @param idx Index of the arc
     * @param horizontalSonarPoints_thetaVals Vector of sampled points from the horizontal sonar beam (arc)
     * @param distributions Vector of beta distributions (probabilities for each arc)
     * @param scanIdx Index of the scan from which the arc (beam) comes from
     * @param arcsText Text (eg arc indexes) to write near each arc (beam) (only if diplaying text)
     * @param modesText Text to write near each arc mode (only if displaying text)
     * @param modes_firstScan Point cloud consisting of all arcs pdf mode for the first scan
     * @param modes_secondScan Point cloud consisting of all arcs pdf mode for the second scan
     */
    void createArcToRender(mrpt::opengl::CPointCloudColouredPtr arcs,
                           int idx,
                           const std::vector<std::vector<scanMerging::pointThetaSimu> > &horizontalSonarPoints_thetaVals,
                           const std::map<int, std::vector<scanMerging::distributionTheta>>& distributions,
                           int scanIdx,
                           mrpt::opengl::CSetOfObjectsPtr arcsText,
                           mrpt::opengl::CSetOfObjectsPtr modesText,
                           mrpt::opengl::CPointCloudPtr modes_firstScan,
                           mrpt::opengl::CPointCloudPtr modes_secondScan);

    /**
     * Render the arcs corresponding to the horizontal sonar measurement. The colors corresponds to the probability distribution
     * More details on the paper :
     * Breux, Yohan, and Lionel Lapierre. "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
     * for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
     * 
     * @param scene Opengl scene
     * @param horizontalSonarPoints_thetaVals Vector of sampled points from the horizontal sonar beam (arc)
     * @param localHorizontalSonarMeasures Vector containing local information on horizontal sonar measure (notably local spherical coords, timestamp)  
     * @param distribution Vector of beta distributions (probabilities for each arc)
     * @param ptToRender Optional vector of indexes to display. If empty, render all arcs.
     */
    void renderArcs(mrpt::opengl::COpenGLScenePtr &scene,
                    const std::vector<std::vector<scanMerging::pointThetaSimu> > &horizontalSonarPoints_thetaVals,
                    const std::vector<scanMerging::horizontalSonarMeasure> &localHorizontalSonarMeasures,
                    const std::map<int, std::vector<scanMerging::distributionTheta> > &distribution,
                    const std::vector<int>& ptToRender = std::vector<int>());

    /** 
     * Render the normals at each scan points (modes of their pdf) relative to the estimated surface
     * For the surface estimation with Gaussian process , see Breux, Yohan, and Lionel Lapierre. "Elevation Angle Estimations of Wide-Beam Acoustic Sonar Measurements 
     * for Autonomous Underwater Karst Exploration." Sensors 20.14 (2020): 4028.
     * For normals computation, see Breux, Yohan, André Mas, and Lionel Lapierre.
     * "On-manifold Probabilistic ICP: Application to Underwater Karst Exploration." (2021) 
     * 
     * @param scene Opengl scene
     * @param horizontalSonarMeasure Vector containing local information on horizontal sonar measure (notably local spherical coords, timestamp)  
     * @param horizontalSonarPoseOnRobot Pose of the horizontal sonar relative to the robot body frame
     * @param res Contains results from the scan merging (using vertical sonar to estimate elevation angle for the horizontal sonar)
    */
    void renderNormals(mrpt::opengl::COpenGLScenePtr& scene,
                       const std::vector<scanMerging::horizontalSonarMeasure>& horizontalSonarMeasure,
                       const mrpt::poses::CPose3D& horizontalSonarPoseOnRobot,
                       const scanMerging::scanMergingResults &res);
    /**
     * Internal function used to render normals (see renderNormals)
     * 
     * @param arcIdx 
     * @param curDistributions 
     * @param horizontalSonarMeasures 
     * @param horizontalSonarPoseOnRobot 
     * @param res 
     * @param normalsArrows 
     * @param boundsCones 
     */
    void renderNormals_(int arcIdx,
                        const std::vector<scanMerging::distributionTheta> &curDistributions,
                        const std::vector<scanMerging::horizontalSonarMeasure> &horizontalSonarMeasures,
                        const mrpt::poses::CPose3D &horizontalSonarPoseOnRobot,
                        const scanMerging::scanMergingResults &res,
                        mrpt::opengl::CSetOfObjectsPtr normalsArrows,
                        mrpt::opengl::CSetOfObjectsPtr boundsCones);
    /**
     * Render one normal
     * 
     * @param curHorizontalSonarMeasure 
     * @param horizontalSonarGlobalPose 
     * @param dist 
     * @param curNormal 
     * @param normalsArrows 
     * @param boundsCones 
     */
    void renderNormal(const scanMerging::horizontalSonarMeasure &curHorizontalSonarMeasure,
                      const mrpt::poses::CPose3D &horizontalSonarGlobalPose,
                      const scanMerging::distributionTheta &dist,
                      const mrpt::poses::CPointPDFGaussian &curNormal,
                      mrpt::opengl::CSetOfObjectsPtr normalsArrows,
                      mrpt::opengl::CSetOfObjectsPtr boundsCones);
    
    /**
     * Generate an opengl cylinder object representing the prior cylinder
     * 
     * @param color 
     * @param name 
     * @return mrpt::opengl::CCylinderPtr 
     */
    mrpt::opengl::CCylinderPtr generatePriorCylinder(const mrpt::utils::TColor& color,
                                                     const std::string& name,
                                                     const scanMerging::ellipticCylinder& cylinder);
    /**
     * Set the pose of prior cylinder opengl object in the scene
     * 
     * @param priorCylinder 
     * @param cylinder 
     * @param cylinderPose 
     */
    void setPriorCylinderPose(mrpt::opengl::CCylinderPtr priorCylinder,
                              const scanMerging::ellipticCylinder &cylinder,
                              const mrpt::poses::CPose3D &cylinderPose);

    /** Render the four mean points used to compute the normal pdf (see Breux, Yohan, André Mas, and Lionel Lapierre.
     * "On-manifold Probabilistic ICP: Application to Underwater Karst Exploration." (2021), section 5.3)
     * 
     * @param scene Opengl scene
     * @param normalPoints Points to be displayed
     */
    void renderNormalPoints_debug(mrpt::opengl::COpenGLScenePtr &scene,
                                  const std::vector<mrpt::math::TPoint3D> &normalPoints);

    void loadParameters_simulation(const mrpt::utils::CConfigFile &cfg);
    void loadParameters_dataSet(const mrpt::utils::CConfigFile &cfg);

    virtual void initScene(const mrpt::utils::CConfigFile &cfg) override;
    virtual void initKeystrokes(const mrpt::utils::CConfigFile& cfg) override;

    bool m_isSimulation; //!< If true, display the mesh used for the simulation

    double m_beamWidth;

    bool m_displayOnlyCurrentScan;
    bool m_stopAtEachRendering;
    bool m_displayUniformArcs;
    bool m_displayNormalUncertainty;
    bool m_displayNormalDebugPoints;
    viewerParameters m_viewerParameters;
};

}} // end namespaces

#endif // DEF_SIMULATION_VIEWER_MRPT_H
