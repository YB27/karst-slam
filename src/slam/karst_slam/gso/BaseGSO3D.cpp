#include "karst_slam/gso/BaseGSO3D.h"
#include "karst_slam/graph/PoseGraph.h"
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::graph;
using namespace karst_slam::gso;

BaseGSO3D::BaseGSO3D(const string& name) :
    RegistrationDeciderOptimizerInterface(name)
{
    init();
}

BaseGSO3D::BaseGSO3D(const string& name,
                     const CConfigFile& configFile):
    RegistrationDeciderOptimizerInterface(name, configFile)
{
    init();
    loadParams(configFile);
}

BaseGSO3D::BaseGSO3D(const string& name,
                     const string& configFile):
    BaseGSO3D(name, CConfigFile(configFile))
{}

BaseGSO3D::~BaseGSO3D()
{}

void BaseGSO3D::init()
{
    m_first_time_call = false;
    m_has_read_config = false;
    m_min_nodes_for_optimization = 4;// default value : 5
    m_last_total_num_of_nodes = 0;
    m_optimization_policy = FOP_USE_LC;
    m_curr_used_consec_lcs = 0;
    m_curr_ignored_consec_lcs = 0;
    m_just_fully_optimized_graph = false;
    m_min_nodes_for_optimization = 3;
}

void  BaseGSO3D::optimizeGraph()
{
    MRPT_START;
    //mrpt::synch::CCriticalSectionLocker m_graph_lock(this->m_graph_section);

    // Originally, always without full update ....
    bool is_full_update = checkForFullOptimization();
     _optimizeGraph(is_full_update);

    //logFmt(LVL_DEBUG, "2nd thread grabbed the lock..");
    MRPT_END;
}

void BaseGSO3D::_optimizeGraph(bool is_full_update)
{
    MRPT_START;
    string funcStr = m_class_name + "::_optimizeGraph";

    // if less than X nodes exist overall, do not try optimizing
    size_t nNode = m_pose_graph->nodeCount();
    if (m_min_nodes_for_optimization > nNode) {
        // At least, convert and compute the marginals
        this->m_pose_graph->updateConvertedGraph();
        this->m_pose_graph->updateMarginals();
        return ;
    }

    CTicTac optimization_timer;
    optimization_timer.Tic();

    // set of nodes for which the optimization procedure will take place
    set<TNodeID>* nodes_to_optimize = nullptr;

    // fill in the nodes in certain distance to the current node, only if
    // is_full_update is not instructed
    if (is_full_update) {
        // nodes_to_optimize: List of nodes to optimize. NULL -> all but the root
        // node.
        nodes_to_optimize = nullptr;
    }
    else {
        nodes_to_optimize = new set<TNodeID>;

        // I am certain that this shall not be called when nodeCount = 0, since the
        // optimization procedure starts only after certain number of nodes has
        // been added
        m_pose_graph->getNearbyNodesOf(nodes_to_optimize,
                                       -1,
                                       nNode-1,
                                       opt_params.optimization_distance);
//        getNearbyNodesOf(nodes_to_optimize,
//                         nNode-1,
//                         opt_params.optimization_distance);
        nodes_to_optimize->insert(nNode-1);
    }

    // Execute the optimization
    execOptimization(nodes_to_optimize);

    m_just_fully_optimized_graph = is_full_update;

    double elapsed_time = optimization_timer.Tac();
    logFmt(LVL_DEBUG,"Optimization of graph took: %fs", elapsed_time);

    // deleting the nodes_to_optimize set
    if(nodes_to_optimize != nullptr)
        delete nodes_to_optimize;
    nodes_to_optimize = nullptr;
    MRPT_UNUSED_PARAM(elapsed_time);
    MRPT_END;
}

template<class THIS_TYPE>
void BaseGSO3D::generateThreadForOptimizeGraph()
{
    m_thread_optimize = mrpt::system::createThreadFromObjectMethod(
                /*obj = */ this,
                /* func = */ &THIS_TYPE::optimizeGraph);
}

bool BaseGSO3D::updateState(const mrpt::obs::CActionCollectionPtr& action,
                            const mrpt::obs::CSensoryFramePtr& observations,
                            const mrpt::obs::CObservationPtr& observation,
                            mrpt::obs::CSensoryFramePtr generatedObservations)
{
    MRPT_START;
    MRPT_UNUSED_PARAM(generatedObservations);
    logFmt(LVL_DEBUG, "In updateOptimizerState... ");

    size_t nNode = m_pose_graph->nodeCount();
    if(nNode > m_last_total_num_of_nodes)
    {
        m_last_total_num_of_nodes = nNode;
        registered_new_node = true;

        // ?????? Never go through this as m_first_time_call is always false ...
        if (m_first_time_call)
        {
            opt_params.last_pair_nodes_to_edge = m_pose_graph->getEdges();
            m_first_time_call = true;
        }
        if (opt_params.optimization_on_second_thread)
        {
            // join the previous optimization thread
            mrpt::system::joinThread(m_thread_optimize);

            // optimize the graph - run on a separate thread
            // Adapted here to use the optimizeGraph function of the inherited class
            generateThreadForOptimizeGraph<typename std::remove_pointer<decltype(this)>::type>();

        }
        else
        { // single threaded implementation
            bool is_full_update = checkForFullOptimization();
            _optimizeGraph(is_full_update);
        }
        return true;
    }

    return false;
    MRPT_END;
}

bool BaseGSO3D::checkForLoopClosures()
{
    MRPT_START;

    bool is_loop_closure = false;
    graph_mrpt_t::edges_map_t curr_pair_nodes_to_edge = m_pose_graph->getEdges();

    // find the *node pairs* that exist in current but not the last nodes_to_edge
    // map If the distance of any of these pairs is greater than
    // LC_min_nodeid_diff then consider this a loop closure
    graph_mrpt_t::edges_map_t::const_iterator search;
    TPairNodeIDs curr_pair;

    for(graph_mrpt_t::edges_map_t::const_iterator it =
        curr_pair_nodes_to_edge.begin(); it != curr_pair_nodes_to_edge.end();
        ++it)
    {
        search = opt_params.last_pair_nodes_to_edge.find(it->first);
        // if current node pair is not found in the last set...
        if (search == opt_params.last_pair_nodes_to_edge.end()) {
            curr_pair = it->first;

            if(abs(static_cast<int>(curr_pair.first) - static_cast<int>(curr_pair.second) ) >
                    opt_params.LC_min_nodeid_diff )
            {
                logFmt(LVL_DEBUG, "Registering loop closure... ");
                is_loop_closure = true;
                break; // no need for more iterations
            }
        }
    }

    // update the pair_nodes_to_edge map
    opt_params.last_pair_nodes_to_edge = curr_pair_nodes_to_edge;
    return is_loop_closure;

    MRPT_END;
}

// ? Same code already in ERD ...
bool BaseGSO3D::checkForFullOptimization()
{
    bool is_full_update = false;

    if (opt_params.optimization_distance == -1) // always optimize fully
        return true;

    bool added_lc = checkForLoopClosures();

    // Decide on the LoopClosingAttitude I am in
    if (!added_lc)
    { // reset both ignored and used counters
        if (m_curr_used_consec_lcs != 0 || m_curr_ignored_consec_lcs != 0)
            MRPT_LOG_DEBUG_STREAM("No new Loop Closure found.");

        m_curr_used_consec_lcs = 0;
        m_curr_ignored_consec_lcs = 0;
        m_optimization_policy = FOP_USE_LC;

        return is_full_update;
    }
    else
    { // lc found.
        // have I used enough consecutive loop closures?
        bool use_limit_reached = (m_curr_used_consec_lcs == m_max_used_consec_lcs);
        // have I ignored enough consecutive loop closures?
        bool ignore_limit_reached = (m_curr_ignored_consec_lcs == m_max_ignored_consec_lcs);

        // Have I reached any of the limits above?
        if (ignore_limit_reached || use_limit_reached)
        {
            m_curr_ignored_consec_lcs = 0;
            m_curr_used_consec_lcs = 0;

            // decide of the my policy on full optimization
            if (ignore_limit_reached)
                m_optimization_policy = FOP_USE_LC;

            if (use_limit_reached)
                m_optimization_policy = FOP_IGNORE_LC;
        }
        else { // no limits reached yet.
            if (m_optimization_policy == FOP_USE_LC)
                m_curr_used_consec_lcs += 1;

            else
                m_curr_ignored_consec_lcs += 1;
        }
    }

    // Decide on whether to fully optimize the graph based on the mode I am in
    if (m_optimization_policy == FOP_IGNORE_LC)
    {
        is_full_update = false;
        MRPT_LOG_WARN_STREAM("*PARTIAL* graph optimization.. ignoring new loop closure");
    }
    else
    {
        is_full_update = true;
        MRPT_LOG_WARN_STREAM("Commencing with *FULL* graph optimization... ");
    }
    return is_full_update;

} // end of checkForFullOptimization

//// ? Same code already in ERD ...
//void BaseGSO3D::getNearbyNodesOf(set<TNodeID> *nodes_set,
//                                 const TNodeID& cur_nodeID,
//                                 double distance )
//{
//    MRPT_START;

//    if(distance > 0)
//    {
//        // check all but the last node.
//        size_t nNode = m_pose_graph->nodeCount();
//        for(TNodeID nodeID = 0; nodeID < nNode; ++nodeID)
//        {
//            double curr_distance = m_pose_graph->getRelativeDistance(nodeID,cur_nodeID);// this->m_graph->nodes[nodeID].distanceTo(this->m_graph->nodes[cur_nodeID]);
//            if (curr_distance <= distance)
//                nodes_set->insert(nodeID);
//        }
//    }
//    else  // check against all nodes
//        *nodes_set = m_pose_graph->getAllNodesId();

//    MRPT_END;
//}


void BaseGSO3D::printParams() const
{
    parent::printParams();
    opt_params.dumpToConsole();
}

void BaseGSO3D::loadOptimizerParams(const mrpt::utils::CConfigFile&  source){opt_params.loadFromConfigFile(source, "OptimizerParameters");}

void BaseGSO3D::loadParams(const CConfigFile& source)
{
    MRPT_START;
    parent::loadParams(source);

    //CConfigFile source(source_fname);

    loadOptimizerParams(source);

    // LC-related info should be computed in ERD and then pass to the GSO
    m_max_used_consec_lcs = source.read_int(
                "OptimizerParameters",
                "max_used_consecutive_loop_closures",
                2, false);

    m_max_ignored_consec_lcs = source.read_int(
                "OptimizerParameters",
                "max_ignored_consecutive_loop_closures",
                15, false);

    // set the logging level if given by the user
    // Minimum verbosity level of the logger
    int min_verbosity_level = source.read_int(
                "OptimizerParameters",
                "class_verbosity",
                1, false);
    setMinLoggingLevel(VerbosityLevel(min_verbosity_level));

    logFmt(LVL_DEBUG, "Successfully loaded Params. ");
    m_has_read_config = true;

    MRPT_END;
}

void BaseGSO3D::getDescriptiveReport(string* report_str) const
{
    MRPT_START;

    const string report_sep(2, '\n');
    const string header_sep(80, '#');

    // Report on graph
    stringstream class_props_ss;
    class_props_ss << m_class_name <<  " Optimization Summary: " << endl;
    class_props_ss << header_sep << endl;

    // time and output logging
    const string output_res = this->getLogAsString();

    // merge the individual reports
    report_str->clear();
    parent::getDescriptiveReport(report_str);

    *report_str += class_props_ss.str();
    *report_str += report_sep;

    *report_str += report_sep;

    *report_str += output_res;
    *report_str += report_sep;

    MRPT_END;
}


// OptimizationParams
//////////////////////////////////////////////////////////////
BaseGSO3D::OptimizationParams::OptimizationParams()
{}

BaseGSO3D::OptimizationParams::~OptimizationParams()
{}

void BaseGSO3D::OptimizationParams::dumpToTextStream(mrpt::utils::CStream &out) const
{
    MRPT_START;

    out.printf("------------------[ General Optimization Parameters ]------------------\n");
    out.printf("Optimization on second thread  = %s\n",
               optimization_on_second_thread ? "TRUE" : "FALSE");
    out.printf("Optimize nodes in distance     = %.2f\n", optimization_distance);
    out.printf("Min. node difference for LC    = %d\n", LC_min_nodeid_diff);

    out.printf("%s", cfg.getAsString().c_str());
    std::cout << std::endl;

    MRPT_END;
}

void BaseGSO3D::OptimizationParams::loadFromConfigFile(const CConfigFileBase &source,
                                                       const string &section)
{
    MRPT_START;
    optimization_on_second_thread = source.read_bool(
                section,
                "optimization_on_second_thread",
                false, false);
    LC_min_nodeid_diff = source.read_int(
                section,
                "LC_min_nodeid_diff",
                30, false);
    optimization_distance = source.read_double(
                section,
                "optimization_distance",
                5, false);
    // asert the previous value
    ASSERTMSG_(optimization_distance == -1 ||
               optimization_distance > 0,
               format("Invalid value for optimization distance: %.2f",
                      optimization_distance) );

    // optimization parameters
    loadFromConfigFile_(source, section);
MRPT_END;
}
