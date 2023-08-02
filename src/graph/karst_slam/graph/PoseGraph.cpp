#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/graph/DirectedTree.h"
#include <mrpt-gtsam/wrapperGTSAM/conversionGraphs.h>
#include <mrpt-gtsam/wrapperGTSAM/gtsamGraph.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::opengl;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::graph;
using namespace karst_slam::nrd;

PoseGraph::PoseGraph() :
    mrpt::utils::COutputLogger("poseGraph"),
    m_graph(make_shared<graph_mrpt_t>()),
    m_graphConverter(make_shared<graph_converter_t>())
{
    setMinLoggingLevel(LVL_DEBUG);

    // By default, add a root node at null pose with index 0
    m_graph->root = 0;
    m_graph->nodes.insert(make_pair(0,pose_t()));
    m_lastNodeID = 0;

    // Link the MRPT graph to the graphConverter to GTSAM graph
    m_graphConverter->setGraphShPtr(m_graph);
}

PoseGraph::PoseGraph(shared_ptr<graph_mrpt_t>&& graph) :
    mrpt::utils::COutputLogger("poseGraph"),
    m_graphConverter(make_shared<graph_converter_t>())
{
    setMinLoggingLevel(LVL_DEBUG);

    ASSERT_(graph != nullptr)
    m_graph = graph;
    
    // Link the MRPT graph to the graphConverter to GTSAM graph
    m_graphConverter->setGraphShPtr(m_graph);

    // Get the last node ID
    set<TNodeID> nodesIDs;
    m_graph->getAllNodes(nodesIDs);
    m_lastNodeID = *nodesIDs.rbegin();
}

PoseGraph::PoseGraph(PoseGraph &&pg)
{
    setMinLoggingLevel(LVL_DEBUG);

    m_graph = pg.m_graph;
    m_graphConverter = pg.m_graphConverter;
    m_lastNodeID = pg.m_lastNodeID;
    m_lastNodePosePdf_atCreation = pg.m_lastNodePosePdf_atCreation;
}

PoseGraph& PoseGraph::operator=(PoseGraph &&pg)
{
    m_graph = pg.m_graph;
    m_graphConverter = pg.m_graphConverter;
    m_lastNodeID = pg.m_lastNodeID;
    m_lastNodePosePdf_atCreation = pg.m_lastNodePosePdf_atCreation;
    return *this;
}

mrpt::utils::TNodeID PoseGraph::getRoot()const
{
    LOCK_GUARD
    return m_graph->root;
}

void PoseGraph::getAllNodesId(set<TNodeID>& nodeIds) const
{
    LOCK_GUARD
    // Caution : Based on node ids appearing in edges !
    m_graph->getAllNodes(nodeIds);
}

void PoseGraph::insertNode_poseRelativeToPrevious(const pose_pdf_t &pose_wrt_lastNode)
{
    LOCK_GUARD
    // Keep the current pose of the node to be inserted (it will certainly be modified later through graph optimization)
    m_lastNodePosePdf_atCreation = (m_lastNodeID == 0) ? pose_wrt_lastNode : getLastNodeEstimatedPosePdf_() + pose_wrt_lastNode;
    m_graph->nodes.emplace_hint(m_graph->nodes.end(), ++m_lastNodeID, m_lastNodePosePdf_atCreation.getMeanVal());
    cout << "[PoseGraph::insertNode_poseRelativeToPrevious] Last node inserted pose : " << m_lastNodePosePdf_atCreation.mean << endl;
    m_lastOdometry               = pose_wrt_lastNode;
}

void PoseGraph::insertNode(const pose_pdf_t &pose_pdf,
                           const pose_pdf_t &lastOdometry)
{
    LOCK_GUARD
    m_graph->nodes.emplace_hint(m_graph->nodes.end(), ++m_lastNodeID,pose_pdf.getMeanVal());
    MRPT_LOG_DEBUG_STREAM("Insert node with id " << m_lastNodeID);
    MRPT_LOG_DEBUG_STREAM(" - pose_pdf : " << pose_pdf);
    cov_mat_t cov = pose_pdf.getCovariance();
    MRPT_LOG_DEBUG_STREAM(" --> cov : " << cov);
    m_lastNodePosePdf_atCreation = pose_pdf;
    m_lastOdometry               = lastOdometry;
}

void PoseGraph::setScanDataToNode(TNodeID nodeId,
                                  const shared_ptr<fullScanProcessedData>& scanData)
{
    m_nodes_to_obs[nodeId] = scanData;   
}

pose_pdf_t PoseGraph::getNodePosePdf_(const TNodeID id) const
{
    return pose_pdf_t(m_graph->nodes.at(id), convertCovGTSAM2MRPT(m_graphConverter->getCovarianceAtNode(id)).inverse());
}

pose_t PoseGraph::getNodePose(const TNodeID id) const
{
    LOCK_GUARD
    return m_graph->nodes.at(id);
}

pose_pdf_t PoseGraph::getNodePosePdf(const TNodeID id) const
{
    LOCK_GUARD
    return getNodePosePdf_(id);
}

pose_pdf_t PoseGraph::getLastNodeEstimatedPosePdf_()const
{
    // Temporary workaround until GTSAM Marginals bug is solved
    // I made an issue in the gtsam github repo : https://github.com/borglab/gtsam/issues/160
    // Edit : It was not a bug, just wrong order :/ (MRPT use [x y z yaw pitch roll] , GTSAM [roll pitch yaw x y z])

    return pose_pdf_t(m_graph->nodes.at(m_lastNodeID), m_lastNodePosePdf_atCreation.cov_inv);
}

pose_pdf_t PoseGraph::getLastNodeEstimatedPosePdf()const
{
    LOCK_GUARD
    return getLastNodeEstimatedPosePdf_();
}

vector<pose_t> PoseGraph::getNodesPose(const set<TNodeID>& nodesId) const
{
    LOCK_GUARD
    vector<pose_t> nodesPose;
    nodesPose.reserve(nodesId.size());

    for(set<TNodeID>::const_iterator it = nodesId.begin(); it != nodesId.end(); it++)
        nodesPose.push_back(m_graph->nodes.at(*it));

    return nodesPose;
}

vector<pose_t> PoseGraph::getNodesPose() const
{
    LOCK_GUARD
    vector<pose_t> nodesPose;
    nodesPose.reserve(m_graph->nodeCount());

    for(nodes_map_t::const_iterator it = m_graph->nodes.begin(); it != m_graph->nodes.end(); it++)
        nodesPose.push_back(it->second);

    return nodesPose;
}

pose_t PoseGraph::getLastNodePose() const
{
    LOCK_GUARD
    return m_graph->nodes.rbegin()->second;
}

pose_pdf_t PoseGraph::getLastNodePosePdf_atCreation() const
{
    LOCK_GUARD
    return m_lastNodePosePdf_atCreation;
}

pose_t PoseGraph::getRelativePose(const TNodeID from, const TNodeID to) const
{
    LOCK_GUARD
    return (m_graph->nodes[to] - m_graph->nodes[from]);
}

pose_pdf_t PoseGraph::getRelativePosePdf(const mrpt::utils::TNodeID from,
                                         const mrpt::utils::TNodeID to)const
{

    // ToDo : modify dijkstra to compute global poses PDF ?
    LOCK_GUARD
    typedef mrpt::graphs::CDijkstra<karst_slam::graph_mrpt_t,
                                    typename karst_slam::graph_mrpt_t::maps_implementation_t> dijkstra_t;

    // ToDo : Very bad to compute full djikstra just to have the relative pose between two nodes ?
    // Not optimized at all, just made to be quickly used
    dijkstra_t dijkstra(*m_graph,from);
    karst_slam::graph::DirectedTree<graph_mrpt_t> treeView;
    dijkstra.getTreeGraph(treeView);

    VisitorComputeRelativePosePdf vs(m_graph.get(),from,to);
    treeView.getRelativePosePdf_bf(from,vs);

    return vs.relativePosePdf;
}

pose_t PoseGraph::getRelativePoseToLastNode(const TNodeID from) const
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Get relative pose between " << from << " and last added node");
    return (m_lastNodePosePdf_atCreation.mean - m_graph->nodes.at(from));
}

double PoseGraph::getRelativeDistance(const TNodeID from, const TNodeID to) const
{
    LOCK_GUARD
    return m_graph->nodes.at(from).distanceTo(m_graph->nodes.at(to));
}

TNodeID PoseGraph::getLastNodeID()const
{
    LOCK_GUARD
    return m_lastNodeID;
}

// ToDo : return the set instead of taking a ptr
void PoseGraph::getNearbyNodesOf(set<TNodeID>* nodes_set,
                                 const TNodeID prev_nodeID,
                                 const TNodeID cur_nodeID,
                                 const double& distanceThreshold) const
{
    LOCK_GUARD
    if(distanceThreshold > 0)
    {
        // check all but the last node.
        size_t nNode = m_graph->nodeCount();
        double curr_distance;
        for(TNodeID nodeID = 0; nodeID < nNode; ++nodeID)
        {
            curr_distance =  m_graph->nodes[nodeID].distanceTo(m_graph->nodes[cur_nodeID]);
            if(curr_distance <= distanceThreshold && nodeID != cur_nodeID && nodeID != prev_nodeID)
                nodes_set->insert(nodeID);
        }
    }
    else  // check against all nodes
        *nodes_set = m_graph->getAllNodes();
}

void PoseGraph::clear()
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Graph is cleared");
    m_graph->clear();
}

size_t PoseGraph::nodeCount() const
{
    LOCK_GUARD
    return m_graph->nodeCount();
}

void PoseGraph::insertEdge(const TNodeID from_nodeID, const TNodeID to_nodeID, const edge_t &edge_value)
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Insert edge between nodes id " << from_nodeID << " and " << to_nodeID);
    MRPT_LOG_DEBUG_STREAM("  - edge constraint : " << edge_value);
    m_graph->insertEdge(from_nodeID, to_nodeID, edge_value);
}

void PoseGraph::insertEdgeAtEnd(const TNodeID from_nodeID, const TNodeID to_nodeID, const edge_t &edge_value)
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Insert edge between nodes id " << from_nodeID << " and " << to_nodeID);
    MRPT_LOG_DEBUG_STREAM("  - edge constraint : " << edge_value);
    m_graph->insertEdgeAtEnd(from_nodeID, to_nodeID, edge_value);
}

PoseGraph::edges_map_t PoseGraph::getEdges() const
{
    LOCK_GUARD
    return m_graph->edges;
}

void PoseGraph::updateConvertedGraph()
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Update the internal gtsam graph");
    m_graphConverter->updateConvertedGraph();
}

pose_pdf_t PoseGraph::getLastOdometry()const
{
    LOCK_GUARD
    return m_lastOdometry;
}

const gtsamGraph& PoseGraph::getConvertedGTSAMGraph() const
{
    // No need to lock here
    return m_graphConverter->getConvertedGTSAMGraph();
}

void PoseGraph::updateMarginals()
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Update marginals");
    m_graphConverter->updateMarginals();
}

void PoseGraph::updateAfterOptimization(const gtsam::Values &newValues)
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Update the internal gtsam graph after optimization");
    m_graphConverter->updateAfterOptimization(newValues);
}

updateInfo PoseGraph::getConvertedGraphLastUpdateInfo() const
{
    LOCK_GUARD
    return m_graphConverter->getLastUpdateInfo();
}

void PoseGraph::dijkstra_nodes_estimate()
{
    LOCK_GUARD
    m_graph->dijkstra_nodes_estimate();
}

void PoseGraph::extractSubGraph(const set<TNodeID> &node_IDs,
                                PoseGraph *sub_graph,
                                const TNodeID root_node_in,
                                const bool &auto_expand_set) const
{
    LOCK_GUARD
    MRPT_LOG_DEBUG_STREAM("Extract subgraph : ");
    MRPT_LOG_DEBUG_STREAM(" - root_node_in      : " << root_node_in);
    MRPT_LOG_DEBUG_STREAM(" - node in subgraphs : ");
    for(const TNodeID& id : node_IDs)
        cout << id << " , ";
    cout << endl;
    shared_ptr<graph_mrpt_t> sub_graph_ptr = make_shared<graph_mrpt_t>();
    m_graph->extractSubGraph(node_IDs, sub_graph_ptr.get(), root_node_in, auto_expand_set);
    PoseGraph pg(move(sub_graph_ptr));
    *sub_graph = move(pg);
}

map<TNodeID, pose_t> PoseGraph::getNodes() const
{
    LOCK_GUARD
    map<TNodeID, pose_t> nodes;
    for(nodes_map_t::const_iterator it = m_graph->nodes.begin(); it != m_graph->nodes.end(); it++)
        nodes[it->first] = it->second;
    return nodes;
}

map<TNodeID,pose_t> PoseGraph::getNodes(const set<TNodeID>& nodesID) const
{
    LOCK_GUARD
    map<TNodeID, pose_t> nodes;
    for(set<TNodeID>::const_iterator it = nodesID.begin(); it != nodesID.end() ; it++)
        nodes[*it] = m_graph->nodes.at(*it);

    return nodes;
}

void PoseGraph::printConvertedGraph() const{m_graphConverter->printGtsamGraph();}