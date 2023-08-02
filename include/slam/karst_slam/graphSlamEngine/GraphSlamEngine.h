#ifndef GRAPHSLAMENGINE_H
#define GRAPHSLAMENGINE_H

#include "karst_slam/typedefs.h"
#include "karst_slam/Observable.h"
#include "karst_slam/nrd/MpIC_NRD3D.h"
#include "karst_slam/erd/MpIC_ERD3D.h"

#include <memory>
#include <vector>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CSensoryFrame.h>

// Forward declarations
namespace mrpt
{
    namespace utils
    {
        class COutputLogger;
        class CConfigFile;
    }
}
namespace karst_slam
{
    namespace gso{class BaseGSO3D;}
    namespace graph{class PoseGraph;}
    namespace nrd{class NodeRegistrationDeciderInterface; class fullScanProcessedData; class MpIC_NRD3D;}
    namespace erd{class EdgeRegistrationDeciderInterface; class MpIC_ERD3D;}
    namespace slam{class OctoMap;}
    namespace sensors{class Sensor;}
}

namespace karst_slam{namespace slam{
/** Main class for managing the Graph Slam */
class GraphSlamEngine : public karst_slam::Observable
{
public:
  using gso_t = karst_slam::gso::BaseGSO3D;
  using nrd_t = karst_slam::nrd::MpIC_NRD3D;
  using erd_t = karst_slam::erd::MpIC_ERD3D;
  using node_to_obs_t = std::map<mrpt::utils::TNodeID, std::shared_ptr<karst_slam::nrd::fullScanProcessedData>>;

  /**
   * Construct a new Graph Slam Engine object
   * 
   * @param config_file  
   * @param sensors Map of <sensor_name, sensors_ptr> on the robot 
   * @param logger 
   */
  GraphSlamEngine(const mrpt::utils::CConfigFile& config_file,
                  const std::map<std::string,std::shared_ptr<karst_slam::sensors::Sensor>>& sensors,
                  const std::shared_ptr<mrpt::utils::COutputLogger>& logger);
  ~GraphSlamEngine();

  /** Main function of this class. Execute one step of the graph-slam algorithm with new measurements
   */
  void executeOneStep(mrpt::obs::CActionCollectionPtr& action,
                      mrpt::obs::CSensoryFramePtr& observations,
                      mrpt::obs::CObservationPtr &observation);

  /**
   * Check if all modules have been correctly initialized
   */
  bool checkInit() const;

  pose_pdf_t getCurrentRobotPose()const;

  inline const std::shared_ptr<nrd_t>& getNRD() const{return m_nrd;}
  inline const std::shared_ptr<erd_t>& getERD() const{return m_erd;}
  inline const std::shared_ptr<gso_t>& getGSO() const{return m_gso;}
  inline bool graphUpdated()const {return m_graphUpdated;}
  inline const std::shared_ptr<karst_slam::graph::PoseGraph>& getPoseGraph() const {return m_pose_graph;}
  inline const std::vector<pose_t>& getOdometryOnlyPoses() const {return m_odometry_only_poses;}
  inline const pose_t& getLastOdometryOnlyPose() const {return m_odometry_only_poses.back();}
  inline const std::vector<pose_t>& getGTPoses() const {return m_gt_poses;}
  inline const std::shared_ptr<OctoMap>& getMap() const {return m_map;}
  inline const karst_slam::scanMerging::scanMergingResults& getLastScanMergingResults() const {return m_erd->getLastScanMergingResults();}
  const node_to_obs_t& getScansPerNode() const;

  mrpt::obs::CActionCollectionPtr getLastActionPtr()const;
  mrpt::obs::CSensoryFramePtr getLastSensoryFramePtr()const;
  mrpt::obs::CObservationPtr getLastObservationPtr() const;
  mrpt::obs::CSensoryFramePtr getLastGeneratedObservationPtr() const;

protected:
  /** Used in constructor to initialize the graphSlamEngine from a configuration file (.ini)
   */
  void init(const mrpt::utils::CConfigFile &config_file,
            const std::map<std::string,std::shared_ptr<karst_slam::sensors::Sensor>>& sensors);

  /**
   * Update the Node Registry Decider (node = topological pose of the robot) 
   * 
   * @param action 
   * @param observations 
   * @param observation 
   * @param generatedObservations 
   * @return True if a node has been added to the graph
   */
  bool updateNRD(mrpt::obs::CActionCollectionPtr &action,
                 mrpt::obs::CSensoryFramePtr &observations,
                 mrpt::obs::CObservationPtr &observation,
                 mrpt::obs::CSensoryFramePtr generatedObservations);

  /**
   * Update the Edge Registry Decider (edge = relative constraint between two nodes)
   * 
   * @param action 
   * @param observations 
   * @param observation 
   * @param generatedObservations 
   * @return True if an edge has been added to the graph
   */
  bool updateERD(mrpt::obs::CActionCollectionPtr &action,
                 mrpt::obs::CSensoryFramePtr &observations,
                 mrpt::obs::CObservationPtr &observation,
                 mrpt::obs::CSensoryFramePtr generatedObservations);

  /**
   * Update the Graph Slam Optimizer (optimize poses at each node to minimize the sum of relative constraints in edges)
   * 
   * @param action 
   * @param observations 
   * @param observation 
   * @return True if optimization done
   */
  bool updateGSO(mrpt::obs::CActionCollectionPtr &action,
                 mrpt::obs::CSensoryFramePtr &observations,
                 mrpt::obs::CObservationPtr &observation);

  /**
   * Attach Scan Data to its corresponding node
   * 
   * @param nodeId 
   * @param scanData 
   */
  void setScanDataToNode(mrpt::utils::TNodeID nodeId,
                         const std::shared_ptr<karst_slam::nrd::fullScanProcessedData>& scanData);

  std::shared_ptr<nrd_t> m_nrd; //!< Module managing the node creation
  std::shared_ptr<erd_t> m_erd; //!< Module managing the edge creation
  std::shared_ptr<gso_t> m_gso; //!< Module managing the graph slam optimization
  std::shared_ptr<karst_slam::graph::PoseGraph> m_pose_graph; //!< Pose Graph
  std::shared_ptr<OctoMap> m_map; //!< Octomap (unused currently)

  std::shared_ptr<mrpt::utils::COutputLogger> m_logger; //!< MRPT logger (for console print)

  mrpt::obs::CActionCollectionPtr m_lastActionPtr; 
  mrpt::obs::CSensoryFramePtr m_lastSensoryFramePtr;
  mrpt::obs::CSensoryFramePtr m_lastGeneratedObservations;
  mrpt::obs::CObservationPtr m_lastObservationPtr;

  std::vector<pose_t> m_odometry_only_poses;
  std::vector<pose_t> m_gt_poses;
  bool m_graphUpdated = false;
  bool m_firstStep = true;
};
}} // end namespaces
#endif //GRAPHSLAMENGINE_H
