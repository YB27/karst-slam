#ifndef GRAPHSLAMENGINE_VIEWER_IMPL_H
#define GRAPHSLAMENGINE_VIEWER_IMPL_H

#include "karst_slam/typedefs.h"
#include "karst_slam/scanMerging/gui/scanMergingViewer_mrpt.h"
#include "karst_slam/nrd/MpIC_NRD3D.h"

#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <map>

// Forward declarations
namespace mrpt
{
    namespace opengl
    {
        class COpenGLScenePtr;
        class CRenderizablePtr;
    }
    namespace obs
    {
        class CSensoryFramePtr;
        class CObservationPtr;
    }
    namespace utils{class CConfigFile;}
    namespace graphslam{class CWindowObserver;}
    namespace gui{class CDisplayWindow3D;}
}

namespace karst_slam{
    namespace graph{class PoseGraph;}
    namespace slam{class GraphSlamEngine;}
    namespace slam{class OctoMap;}
    namespace gui{class SensorViewer;class MSISViewer;}
    namespace obs{class ObservationMSISBeamPtr;}
}

namespace karst_slam{namespace gui{
class GraphSlamEngineViewer_impl : public simulationViewer_mrpt 
{

  using node_to_obs_t = std::map<uint64_t, std::shared_ptr<karst_slam::nrd::fullScanProcessedData>>;

  public:
    using sensorLabel_t = std::string;

    explicit GraphSlamEngineViewer_impl(const std::string& name = "GraphSlamEngineViewer");
    ~GraphSlamEngineViewer_impl() = default;

    /******************************************************/
    /********************INIT Things***********************/
    /******************************************************/
    void initKeystrokes(const mrpt::utils::CConfigFile& cfg) override;

    void initScene(const mrpt::utils::CConfigFile& cfg) override;

    void initMSISViewers(mrpt::opengl::COpenGLScenePtr& scene, 
                         const mrpt::utils::CConfigFile& cfg);

    void initMapDisplay(mrpt::opengl::COpenGLScenePtr& scene, 
                        const mrpt::utils::CConfigFile& cfg);


    void initOdometryOnlyDisplay(mrpt::opengl::COpenGLScenePtr& scene, 
                                const mrpt::utils::CConfigFile& cfg);

    void initGroundTruthDisplay(mrpt::opengl::COpenGLScenePtr& scene, 
                                const mrpt::utils::CConfigFile& cfg);

    void initPoseGraphDisplay(mrpt::opengl::COpenGLScenePtr& scene,
                             const mrpt::utils::CConfigFile& cfg);

    mrpt::opengl::CRenderizablePtr initRobotModel(const mrpt::utils::CConfigFile& cfg);

    void initFixedObjects(mrpt::opengl::COpenGLScenePtr& scene, 
                          const mrpt::utils::CConfigFile& cfg);

    void createMiscObjects(mrpt::opengl::COpenGLScenePtr& scene,
                           const std::string& sensorName,
                           const std::string& sensorType,
                           const mrpt::utils::CConfigFile& cfg);

    void createMiscObjects_MSIS(mrpt::opengl::COpenGLScenePtr& scene,
                                const std::string& sensorName,
                                const mrpt::utils::CConfigFile& cfg);


    /******************************************************/
    /********************Rendering Things******************/
    /******************************************************/
    void render(karst_slam::slam::GraphSlamEngine* engine);

    void renderPoseGraph(mrpt::opengl::COpenGLScenePtr& scene, 
                         const std::shared_ptr<karst_slam::graph::PoseGraph>& pose_graph);

    void renderPointCloudPoses(mrpt::opengl::COpenGLScenePtr& scene, 
                               const std::vector<pose_t>& poses,
                               const std::string& objName);

    void renderOdometryOnlyPoses(mrpt::opengl::COpenGLScenePtr& scene, 
                                 const pose_t &lastOdoPose,size_t nOdo);

    void renderGTPoses(mrpt::opengl::COpenGLScenePtr& scene, 
                       const std::vector<pose_t>& gtPoses);

    void renderMap(mrpt::opengl::COpenGLScenePtr& scene, 
                  const std::shared_ptr<karst_slam::slam::OctoMap>& map);

    void renderScans(mrpt::opengl::COpenGLScenePtr& scene,
                     const node_to_obs_t& obs,
                     const std::shared_ptr<graph::PoseGraph> &poseGraph);

    void renderLastObservations(mrpt::opengl::COpenGLScenePtr& scene,
                                const mrpt::obs::CSensoryFramePtr& observations,
                                const mrpt::obs::CObservationPtr& observation,
                                const mrpt::obs::CSensoryFramePtr& generatedObservations,
                                const pose_pdf_t &currentRobotPose);

    void renderObservation(mrpt::opengl::COpenGLScenePtr& scene,
                           const mrpt::obs::CObservationPtr& observation,
                           const pose_pdf_t &currentRobotPose);

    void renderMSIS_frustum(mrpt::opengl::COpenGLScenePtr& scene,
                            const karst_slam::obs::ObservationMSISBeamPtr& obs_beam,
                            const mrpt::poses::CPose3D &currentRobotPose);

    void renderRobotEstPose(mrpt::opengl::COpenGLScenePtr& scene,
                            const pose_pdf_t &lastRobotEstPose);

    void renderScanEstimatedData(mrpt::opengl::COpenGLScenePtr& scene,
                                 const mrpt::poses::CPose3D& horizontalSonarPoseOnRobot,
                                 const std::vector<std::vector<scanMerging::pointThetaSimu>>& horizontalSonarPoints_thetaVals,
                                 const std::vector<scanMerging::horizontalSonarMeasure>& horizontalSonarMeasures,
                                 const scanMerging::surfaceTrainingData& td,
                                 const scanMerging::ellipticCylinder& priorCylinder,
                                 const scanMerging::scanMergingResults& scanRes);

    inline bool isExit()const {return m_isExit;}
    inline bool isPause()const {return m_isPause;}
    inline void exit() {std::cout << "Exit requested" << std::endl;m_isExit = true;}
    inline void pause() {m_isPause = true;}
    inline void resume() {m_isPause = false;}


  private:
    std::shared_ptr<karst_slam::slam::GraphSlamEngine> m_enginePtr;    
    std::shared_ptr<MSISViewer> m_verticalSonarViewer;
    std::shared_ptr<MSISViewer> m_horizontalSonarViewer; 

    bool m_isExit   = false;
    bool m_isPause  = false;
    bool m_displayVerticalSonarMaxLimit = false;
    size_t m_prev_nOdo;
};
}} // end namespaces

#endif // GRAPHSLAMENGINE_VIEWER_IMPL_H
