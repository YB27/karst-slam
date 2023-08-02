#include "karst_slam/graphSlamEngine/gui/GraphSlamEngineViewer.h"
#include "karst_slam/graphSlamEngine/GraphSlamEngine.h"
#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/maps/Octomap.h"
//#include "karst_slam/sensors/MSIS.h"
#include "karst_slam/obs/ObservationMSISBeam.h"
#include "karst_slam/sensors/SensorFactory.h"

#include <mrpt/system/threads.h>
#include <mrpt/maps/CSimpleMap.h>

#include <mrpt/utils/CConfigFile.h>
#include <pid/rpath.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::graph;
using namespace karst_slam::gui;
using namespace karst_slam::slam;
using namespace karst_slam::obs;
using namespace karst_slam::sensors;

vector<pose_t> generateDummyGT(int nPoint)
{
    vector<pose_t> gtPoses;
    gtPoses.reserve(nPoint);
    for(int i = 0; i < nPoint; i++)
        gtPoses.push_back(mrpt::poses::CPose3D(i,i,i));

    return gtPoses;
}

vector<pose_t> generateDummyOdometry(int nPoint)
{
    vector<pose_t> odoPoses;
    odoPoses.reserve(nPoint);
    for(int i = 0; i < nPoint; i++)
        odoPoses.push_back(mrpt::poses::CPose3D(i,-i,i));

    return odoPoses;
}

shared_ptr<PoseGraph> generateDummyPoseGraph(int nPoint)
{
    shared_ptr<graph_mrpt_t> mrptGraph = make_shared<graph_mrpt_t>();
    mrptGraph->nodes.insert(make_pair(0, mrpt::poses::CPose3D(0,0,0)));
    for(int i = 1; i < nPoint; i++)
    {
        mrptGraph->nodes.insert(make_pair(i, mrpt::poses::CPose3D(-i, i,-i)));
        mrptGraph->insertEdge(i-1, i, mrpt::poses::CPose3D(1,1,1));
    }
    return make_shared<PoseGraph>(move(mrptGraph));
}

shared_ptr<OctoMap> generateDummyMap(const CConfigFile& cfg)
{
    shared_ptr<OctoMap> map = make_shared<OctoMap>(cfg);
    mrpt::maps::CSimpleMap simpleMap;
    simpleMap.loadFromFile(PID_PATH("testSimpleMap.gz"));
    map->loadFromSimpleMap(simpleMap);

    return map;
}

shared_ptr<MSIS_primary> generateDummyMSIS(const CConfigFile& cfg)
{
    shared_ptr<MSIS_primary> msis = make_shared<MSIS_primary>("dummyMSIS");
    msis->loadParams(cfg);

    // Assume the sonar move parallel to a wall at a distance d
    float d = 10.;
    MSISParams params = msis->getParams();
    float anglePerStep = msis->getAnglePerStep(), curAngle = 0.;
    mrpt::poses::CPose3DPDFGaussian curBeamGlobalPose, localIncr;
    curBeamGlobalPose.mean = mrpt::poses::CPose3D(1.,1.,1.,0.6,0.,0.);
    curBeamGlobalPose.cov  = mrpt::math::CMatrixDouble66();
    localIncr.mean         = mrpt::poses::CPose3D(0.1,0.,0.,0.,0.,0.);
    localIncr.cov          = mrpt::math::CMatrixDouble66();
    localIncr.cov(2,2)     = 1e-6; // To avoid zero covariance for points along the z axis (ellipsoid not drawn in this case)
    for(int i = 0; i < params.nAngleStep; i++)
    {
        ObservationMSISBeamPtr b = ObservationMSISBeam::Create("dummyName");
        curBeamGlobalPose += localIncr;
        b->setAngle(curAngle);
        //b.setRelativePoseToPreviousBeam(localIncr);
        //b.setGlobalPose(curBeamGlobalPose);

        // Scan only see the walls for angle in [0,pi]
        vector<int> intensities(params.nIntensityBins,0);
        if(curAngle > 1e-3 && (M_PI - curAngle) > 1e-3)
        {
            float distanceToWall = d/sin(curAngle);
            if(distanceToWall <= params.maxRange)
            {
                int idx = round(distanceToWall/params.intensityStep);
                intensities[idx] = 100;
            }
        }
        b->setIntensities(intensities);
        msis->addBeam(b,localIncr,curBeamGlobalPose,mrpt::poses::CPose3DPDFGaussian(),true);

        curAngle += anglePerStep;
    }


    return msis;
}

class dummyGraphSlamEngine : public GraphSlamEngine
{
  public:
    dummyGraphSlamEngine(const CConfigFile& config_file,
                         const std::map<std::string,std::shared_ptr<karst_slam::sensors::Sensor>>& sensors,
                         const std::shared_ptr<mrpt::utils::COutputLogger>& logger) : GraphSlamEngine(config_file,
                                                                                                      sensors,
                                                                                                      logger){}

    ~dummyGraphSlamEngine(){}
    void setPoseGraph(const shared_ptr<PoseGraph>& pg){m_pose_graph = pg;}
    void setGTPoses(const vector<pose_t>& gtposes){m_gt_poses = gtposes;}
    void setOdoPoses(const vector<pose_t>& odoposes){m_odometry_only_poses = odoposes;}
    void setMap(const shared_ptr<OctoMap>& map){m_map = map;}
};


int testDisplay()
{
    string cfg_name = PID_PATH("test_config_file.ini");
    CConfigFile cfg(cfg_name);
    GraphSlamEngineViewer gsv(cfg);

    // Dummy data
    map<string,shared_ptr<Sensor>> sensors = karst_slam::sensors::createSensors(cfg);

    shared_ptr<COutputLogger> logger = make_shared<COutputLogger>();
    dummyGraphSlamEngine engine(cfg,sensors,logger);
    int nPoint = 50;
    engine.setOdoPoses(generateDummyOdometry(nPoint));
    //engine.setGTPoses(generateDummyGT(nPoint));
    engine.setPoseGraph(generateDummyPoseGraph(nPoint));
    //engine.setMap(generateDummyMap(cfg));

    // ToDo : make for all observations
//    shared_ptr<MSIS> msis = generateDummyMSIS(cfg);
//    msis->dumpParamsToConsole();
//    engine.setMSIS(msis);

    // Remind the current shortcuts
    gsv.dumpKeystrokesToConsole();

    // Render
//    mrpt::obs::CSensoryFramePtr sf;
//    mrpt::obs::CObservationPtr obs;
    gsv.updateFrom(&engine);//, sf, obs);

//    // Test exit (keystroke defined in the config file. Default : Ctrl + c)
    while(!gsv.isExit())
    {
        gsv.queryKeyboardEvents();
        mrpt::system::sleep(500);
    }
    return 0;
}

