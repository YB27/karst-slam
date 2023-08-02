#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/utils/COutputLogger.h>
#include <mutex>
#include <atomic>
#include <map>
#include <set>
#include <memory>
#include "karst_slam/typedefs.h"

#define LOCK_GUARD std::lock_guard<std::mutex> lk(m_mutex);

// Forward decl
namespace mrpt_gtsam
{
    namespace wrapperGTSAM
    {
        template<class T> class mrptToGTSAM_graphConverter;
        class gtsamGraph;
        class updateInfo;
    }
}
namespace mrpt
{
    namespace poses
    {
        class CPose3DPDFGaussianInf;
        class CPose3DQuatPDFGaussianInf;
        class CPose3D;
        class CPose3DQuat;
    }
    namespace opengl{class CSetOfObjectsPtr;}
}

namespace gtsam{class Values;}

namespace karst_slam{namespace nrd{class fullScanProcessedData;}}

namespace karst_slam{namespace graph{
/**
 * Thread-safe interface for CNetworkOfPoses3DInf
 * Interface to a mrpt graph slam.
 * Note that when using an ISAM GSO (graph slam optimizer), it keeps a converted version 
 * of the mrpt pose graph in gtsam format.
 * 
 * Warning : The converted gtsam is only updated by the GSO module !
 * When used independently (for tests, etc ...), need to manually call updateConvertedGraph().
 */
class PoseGraph : public mrpt::utils::COutputLogger
{
public:
    // Useful typedef
    using edge_t       = graph_mrpt_t::edge_t;//mrpt::poses::CPose3DPDFGaussianInf;
    using edges_map_t  = graph_mrpt_t::edges_map_t; //mrpt::aligned_containers<std::pair<uint64_t,uint64_t>,edge_t>::multimap_t;//graph_mrpt_t::edges_map_t;
    using nodes_map_t  = graph_mrpt_t::global_poses_t;//std::map<uint64_t, graph_mrpt_t::global_pose_t>;
    using graph_converter_t =  mrpt_gtsam::wrapperGTSAM::mrptToGTSAM_graphConverter<graph_mrpt_t>;
    using nodes_to_scans_t = std::map<mrpt::utils::TNodeID, std::shared_ptr<karst_slam::nrd::fullScanProcessedData>>;

    /** Default ctor */
    PoseGraph();

    /** Constructor from an mrpt graph */
    explicit PoseGraph(std::shared_ptr<graph_mrpt_t> &&graph); // ToDo :: Use unique_ptr ?
    
    /** Move ctor */
    explicit PoseGraph(PoseGraph&& pg);
    
    /** Dtor */
    virtual ~PoseGraph() = default;

    PoseGraph& operator=(PoseGraph&& pg);

    /** Get the root node idx (generally 0) */
    mrpt::utils::TNodeID getRoot()const;

    /** Get the set of all graph nodes ids */
    void getAllNodesId(std::set<mrpt::utils::TNodeID>& nodesIDs) const;

    /** Insert a node (pose pdf) in the graph by providing its initial pose pdf
     * 
     * @param pose_pdf initial pose pdf value of the node to insert
     * @param lastOdometry last odometry (relative displacement) leading to this node
    */
    void insertNode(const pose_pdf_t &pose_pdf,const pose_pdf_t &lastOdometry);

    /** Insert a node (pose pdf) in the graph by providing a relative pose to the last added node 
     * 
     * @param pose_wrt_lastNode pose pdf of the node to add expressed relatively to the last added node
    */
    void insertNode_poseRelativeToPrevious(const pose_pdf_t &pose_wrt_lastNode);

    /** Attach scan data (from sonar sensors)  to a node 
     * 
     * @param nodeId node on which to attach scan data
     * @param scanData shared ptr of scan data to be attached
    */
    void setScanDataToNode(mrpt::utils::TNodeID nodeId,
                           const std::shared_ptr<karst_slam::nrd::fullScanProcessedData>& scanData);
    
    pose_t getNodePose(const mrpt::utils::TNodeID id) const;
    pose_pdf_t getNodePosePdf(const mrpt::utils::TNodeID id) const;
    std::vector<pose_t> getNodesPose(const std::set<mrpt::utils::TNodeID>& nodesId) const;
    std::vector<pose_t> getNodesPose() const;
    pose_t getLastNodePose() const;
    pose_pdf_t getLastNodePosePdf_atCreation() const;
    pose_pdf_t getLastNodeEstimatedPosePdf()const;

    pose_t getRelativePose(const mrpt::utils::TNodeID from,
                           const mrpt::utils::TNodeID to) const;
    pose_pdf_t getRelativePosePdf(const mrpt::utils::TNodeID from,
                                  const mrpt::utils::TNodeID to)const;

    pose_t getRelativePoseToLastNode(const mrpt::utils::TNodeID from)const;

    double getRelativeDistance(const mrpt::utils::TNodeID from,
                               const mrpt::utils::TNodeID to) const;

    /**　Get a set of nodes idx under a threshold distance of a given node
     * 
     *  @param nodes_set [out] set of nearby nodes
     *  @param prev_nodeID previous nodes id (ignored from the nearby set)
     *  @param cur_nodeID considered node
     *  @param distanceThreshold Maximum distance between the considered node and the nearby set 
    */
    void getNearbyNodesOf(std::set<mrpt::utils::TNodeID>* nodes_set,
                          const mrpt::utils::TNodeID prev_nodeID,
                          const mrpt::utils::TNodeID cur_nodeID,
                          const double& distanceThreshold) const;

    pose_pdf_t getLastOdometry()const;

    size_t nodeCount() const;
    mrpt::utils::TNodeID getLastNodeID()const;
    
    /** Insert an edge in the graph (relative pose between two nodes (topological poses)) */
    void insertEdge(const mrpt::utils::TNodeID from_nodeID, 
                    const mrpt::utils::TNodeID to_nodeID, 
                    const edge_t &edge_value);

    /** Insert an edge at the end of the graph */               
    void insertEdgeAtEnd(const mrpt::utils::TNodeID from_nodeID, 
                         const mrpt::utils::TNodeID to_nodeID, 
                         const edge_t &edge_value);
    edges_map_t getEdges() const;

    /** Get the underlayered equivalent gtsam graph */
    const mrpt_gtsam::wrapperGTSAM::gtsamGraph& getConvertedGTSAMGraph() const;
    mrpt_gtsam::wrapperGTSAM::updateInfo getConvertedGraphLastUpdateInfo() const;

    void clear();

    void extractSubGraph(const std::set<mrpt::utils::TNodeID>& node_IDs,
                         PoseGraph* sub_graph,
                         const mrpt::utils::TNodeID root_node_in=INVALID_NODEID,
                         const bool& auto_expand_set=true) const;

    std::map<mrpt::utils::TNodeID, pose_t> getNodes() const;
    std::map<mrpt::utils::TNodeID, pose_t> getNodes(const std::set<mrpt::utils::TNodeID>& nodesID) const;

    inline const nodes_to_scans_t& getNodesToScans() const {return m_nodes_to_obs;}

    // Functions related to convertion into gtsam graph
    /** Update the converted gtsam graph to be equivalent to the wrapped mrpt graph 
     * Note that this is generally called by the GSO module as gtsam graph is only
     * used during optimization with ISAM algorithms. 
     * There is no need to update this graph continuously.
    */
    void updateConvertedGraph();
    void updateMarginals();
    void updateAfterOptimization(const gtsam::Values& newValues);

    void dijkstra_nodes_estimate();

    /** Dump the converted graph to console (for debug) */
    void printConvertedGraph()const;

protected:
    pose_pdf_t getLastNodeEstimatedPosePdf_()const;
    inline pose_pdf_t getNodePosePdf_(const mrpt::utils::TNodeID id) const;

    // ToDo : Use std::atomic when possible
    mutable std::mutex m_mutex;
    nodes_to_scans_t m_nodes_to_obs; //!<  Map relating nodes and scan data
    std::shared_ptr<graph_mrpt_t> m_graph; //!< MRPT graph
    std::shared_ptr<graph_converter_t> m_graphConverter; //!< Converter MRPT<->GTSAM graph
    mrpt::utils::TNodeID m_lastNodeID;
    pose_pdf_t m_lastNodePosePdf_atCreation; ///< Temporary keep the information matrix of the last added node. Need it before the marginals are computed.
    pose_pdf_t m_lastOdometry;//!< Keep the last odometry measurement between the two last added nodes. It will be reused for scan matching.
    pose_pdf_t m_lastNodeEstimatedPosePdf; 
};
}} // end namespaces

#endif // POSEGRAPH_H
