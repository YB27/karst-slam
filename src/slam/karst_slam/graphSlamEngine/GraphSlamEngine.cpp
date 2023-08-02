#include "karst_slam/graphSlamEngine/GraphSlamEngine.h"
#include "karst_slam/graphSlamEngine/moduleLoader.h"
#include "karst_slam/maps/Octomap.h"
#include "karst_slam/obs/ObservationMSIS_scan.h"

#include <mrpt/utils/CConfigFile.h>

#include <memory>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::graph;
using namespace karst_slam::slam;
using namespace karst_slam::obs;
using namespace karst_slam::nrd;
using node_to_obs_t = map<TNodeID, shared_ptr<fullScanProcessedData>>;

GraphSlamEngine::GraphSlamEngine(const CConfigFile &config_file,
                                 const map<string,shared_ptr<sensors::Sensor>>& sensors,
                                 const shared_ptr<COutputLogger>& logger) :
                                m_logger(logger),
                                m_nrd(nullptr),
                                m_erd(nullptr),
                                m_gso(nullptr),
                                m_map(make_shared<OctoMap>(config_file))
{
  m_pose_graph = make_shared<PoseGraph>();
  init(config_file, sensors);
}

GraphSlamEngine::~GraphSlamEngine()
{}


void GraphSlamEngine::init(const CConfigFile &config_file,
                           const map<string,shared_ptr<sensors::Sensor>>& sensors)
{
   // Load the node/edge registration decider (NRD/ERD) and graph Optimizer classes
   loadModule(config_file, "nrd_class", "ICPCriteriaNRD3D", sensors, m_nrd);
   cout << "[GraphSlamEngine::init] NRD module loaded " << endl;
   loadModule(config_file, "erd_class", "ICPCriteriaERD3D", sensors, m_erd);
   cout << "[GraphSlamEngine::init] ERD module loaded " << endl;
   loadModule(config_file, "gso_class", "GaussNewtonGSO3D", sensors, m_gso);
   cout << "[GraphSlamEngine::init] GSO module loaded " << endl;

   // ToDo : load GT poses
    
   // Pass the graph ptr to the modules
   m_nrd->setPoseGraphPtr(m_pose_graph);
   m_erd->setPoseGraphPtr(m_pose_graph);
   m_gso->setPoseGraphPtr(m_pose_graph);
   
   m_erd->setMSISparams(static_pointer_cast<sensors::MSIS>(sensors.at("Sonar_Seaking"))->getParams(), 
                        static_pointer_cast<sensors::MSIS>(sensors.at("Sonar_Micron"))->getParams());
}

bool GraphSlamEngine::checkInit()const
{
    return (m_nrd->checkInit() && m_erd->checkInit() && m_gso->checkInit());
}

pose_pdf_t GraphSlamEngine::getCurrentRobotPose() const {return m_nrd->getCurrentRobotPosEstimation();}

void GraphSlamEngine::executeOneStep(CActionCollectionPtr &action,
                                     CSensoryFramePtr &observations,
                                     CObservationPtr &observation)
{
    MRPT_START
    // NRD
    // Remember that odometry only poses are here !
    CSensoryFramePtr generatedObservations = CSensoryFrame::Create();
    bool registered_new_node = updateNRD(action, observations, observation, generatedObservations);
    if(registered_new_node)
    {   
        cout << "[GraphSlamEngine::executeOneStep] Ok add scan to erd" << endl; 
        m_erd->setLastProcessedScan(m_nrd->getFullScanProcessedData());            
    }
    if(m_firstStep)
        m_firstStep = false;

    // ERD
    updateERD(action, observations, observation, generatedObservations);

    // GSO
    m_graphUpdated = false;
    // m_graphUpdated = updateGSO(action, observations, observation);

    // Criteria for the graph construction ?
    if(registered_new_node)
    {
        m_graphUpdated = true;

        // TODO
        // Use dijkstra nodes estimate only for nodes which were not optimized
        // Otherwise, it will overwrite the optimized values.
        //m_pose_graph->dijkstra_nodes_estimate();

//        // Update map
//        //m_map->update(m_nodes_to_obs, m_pose_graph->getNodes());
    }

    m_lastActionPtr.copy(action);
    m_lastSensoryFramePtr.copy(observations);
    m_lastObservationPtr.copy(observation);
    m_lastGeneratedObservations.copy(generatedObservations);

    // Only update UI, etc.. when a new node has been registered  
    if(registered_new_node)
    {
        notify();
        mrpt::system::pause();
    }

    MRPT_END
}

bool GraphSlamEngine::updateNRD(CActionCollectionPtr &action,
                                CSensoryFramePtr &observations,
                                CObservationPtr &observation,
                                CSensoryFramePtr generatedObservations)
{
    //m_logger->logFmt(LVL_DEBUG,"Update NRD ...");
    bool registered_new_node = m_nrd->updateState(action, observations, observation, generatedObservations);
    //m_logger->logFmt(LVL_DEBUG,"Node registered ? %i",registered_new_node);
    m_odometry_only_poses.push_back(m_nrd->getCurrentOdometryOnlyPose());
    return registered_new_node;
}

bool GraphSlamEngine::updateERD(CActionCollectionPtr &action,
                                CSensoryFramePtr &observations,
                                CObservationPtr &observation,
                                CSensoryFramePtr generatedObservations)
{
    return m_erd->updateState(action, observations, observation,generatedObservations);
}

bool GraphSlamEngine::updateGSO(CActionCollectionPtr &action,
                                CSensoryFramePtr &observations,
                                CObservationPtr &observation)
{
    return m_gso->updateState(action, observations, observation);
}

const node_to_obs_t& GraphSlamEngine::getScansPerNode() const {return m_pose_graph->getNodesToScans();}

CActionCollectionPtr GraphSlamEngine::getLastActionPtr() const{return m_lastActionPtr;}

CSensoryFramePtr GraphSlamEngine::getLastSensoryFramePtr()const {return m_lastSensoryFramePtr;}

CObservationPtr GraphSlamEngine::getLastObservationPtr()const {return m_lastObservationPtr;}

CSensoryFramePtr GraphSlamEngine::getLastGeneratedObservationPtr()const {return m_lastGeneratedObservations;}
