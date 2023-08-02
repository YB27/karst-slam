#ifndef BASEGSO3D_H
#define BASEGSO3D_H

#include "karst_slam/typedefs.h"
#include "karst_slam/interfaces/RegistrationDeciderOrOptimizerInterface.h"
#include <mrpt/utils/CLoadableOptions.h>
#include <mrpt/system/threads.h>

#include <iostream>
#include <functional>
#include <string>
#include <map>
#include <cmath>
#include <type_traits>

// Forward declarations
namespace mrpt
{
    namespace obs{class CSensoryFramePtr;}
    namespace utils{class CStream;}
}

namespace karst_slam{namespace gso{


/** Clean-up version (separate visual-related stuffs,...) of CBaseGSO from mrpt-gtsam in 3D case only
 */
class BaseGSO3D : public karst_slam::slam::RegistrationDeciderOptimizerInterface
{
public :
    /** \brief Handy typedefs */
    /** \{*/
    using InfMat = mrpt::math::CMatrixFixedNumeric<double,
    pose_pdf_t::state_length,
    pose_pdf_t::state_length>;
    using parent = RegistrationDeciderOptimizerInterface;
    /** \}*/

    explicit BaseGSO3D(const std::string& name);
    BaseGSO3D(const std::string& name, const std::string& configFile);
    BaseGSO3D(const std::string& name, const mrpt::utils::CConfigFile& configFile);
    virtual ~BaseGSO3D();

    struct OptimizationParams: public mrpt::utils::CLoadableOptions
    {
    public:
        OptimizationParams();
        ~OptimizationParams();

        void loadFromConfigFile(
                const mrpt::utils::CConfigFileBase &source,
                const std::string &section = "OptimizerParameters");
        void dumpToTextStream(mrpt::utils::CStream &out) const;

        // Parameters specific to the optimizer
        mrpt::utils::TParametersDouble cfg;

        // True if optimization procedure is to run in a multithreading fashion
        bool optimization_on_second_thread = false; // currently not working

        /** \brief optimize only for the nodes found in a certain distance from
          * the current position. Optimize for the entire graph if set to -1
          */
        double optimization_distance;
        double offset_y_optimization_distance;


        // nodeID difference for an edge to be considered loop closure
        int LC_min_nodeid_diff;

        // Map of TPairNodesID to their corresponding edge as recorded in the
        // last update of the optimizer state
        graph_mrpt_t::edges_map_t last_pair_nodes_to_edge;

    protected:
        virtual void loadFromConfigFile_(
                const mrpt::utils::CConfigFileBase &source,
                const std::string &section="OptimizerParameters")
        {std::cout << " !! Empty implementation of CBaseGSO::loadFromConfigFile_ !! " << std::endl;}
    };

    virtual bool updateState(const mrpt::obs::CActionCollectionPtr& action,
                             const mrpt::obs::CSensoryFramePtr& observations,
                             const mrpt::obs::CObservationPtr& observation,
                             mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr()) override;


    // For optimizer/visualization parameters
    virtual void loadParams(const mrpt::utils::CConfigFile& source) override;
    virtual void printParams() const override;

    // State of the optimizer
    virtual void getDescriptiveReport(std::string* report_str) const override;

    inline bool justFullyOptimizedGraph() const {return m_just_fully_optimized_graph;}

    void init();

    inline void setMinNodesToOptimize(size_t n){m_min_nodes_for_optimization = n;}

    virtual bool checkInit_()const override{return true;}

    /** Parameters relevant to the optimizatio nfo the graph. */
    OptimizationParams opt_params;
protected :
    virtual inline void loadOptimizerParams(const mrpt::utils::CConfigFile&  source);//{opt_params.loadFromConfigFile(source, "OptimizerParameters");}

    /** \brief Wrapper around createThreadFromObjectMethod
     *
     */
    template<class THIS_TYPE>
    void generateThreadForOptimizeGraph();

    virtual void optimizeGraph();
    /** \brief Optimize the given graph.
      *
      * Wrapper around the graph optimizer method called by execOptimization
      * \sa optimizeGraph, execOptimization
      *
      * \param[in] full_update Impose that method optimizes the whole graph
      *
      */
    virtual void _optimizeGraph(bool is_full_update=false);

    virtual void execOptimization(const std::set<uint64_t>* nodes_to_optimize = nullptr) = 0;

    /** \brief Check if a loop closure edge was added in the graph.
      *
      * Match the previously registered edges in the graph with the current. If
      * there is a node difference *in any new edge* greater than
      * \b LC_min_nodeid_diff (see .ini parameter) then new constraint is
      * considered a Loop Closure
      *
      * \return True if \b any of the newly added edges is considered a loop
      * closure
      */
    virtual bool checkForLoopClosures();

    /** \brief Decide whether to issue a full graph optimization
      *
      * In case N consecutive full optimizations have been issued, skip some of
      * the next as they slow down the overall execution and they don't reduce
      * the overall error
      *
      * \return True for issuing a full graph optimization, False otherwise
      */
    virtual bool checkForFullOptimization();

    /** \brief Indicates whether a full graph optimization was just issued.
     */
    bool m_just_fully_optimized_graph;

    bool m_first_time_call;
    bool m_has_read_config;
    bool registered_new_node;
    size_t m_min_nodes_for_optimization;

    // Start optimizing the graph after a certain number of nodes has been
    // added (when m_graph->nodeCount() > m_last_total_num_of_nodes)
    size_t m_last_total_num_of_nodes;

    // Use second thread for graph optimization
    mrpt::system::TThreadHandle m_thread_optimize;

    /** \brief Enumeration that defines the behaviors towards using or ignoring a
      * newly added loop closure to fully optimize the graph
      */
    enum FullOptimizationPolicy {
        FOP_IGNORE_LC=0,
        FOP_USE_LC,
        FOP_TOTAL_NUM
    };
    /** \brief Should I fully optimize the graph on loop closure?
      */
    FullOptimizationPolicy m_optimization_policy;

    /** \name Smart Full-Optimization Command
      *
      * Instead of issuing a full optimization every time a loop closure is
      * detected, ignore current loop closure when enough consecutive loop
      * closures have already been utilised.
      * This avoids the added computational cost that is needed for optimizing
      * the graph without reducing the accuracy of the overall operation
      */
    /** \{*/

    /** \brief Number of maximum cosecutive loop closures that are allowed to be
      * issued.
      *
      * \sa m_curr_used_consec_lcs, m_max_ignored_consec_lcs
      */
    size_t m_max_used_consec_lcs;

    /** \brief Number of consecutive loop closures that are currently registered
      *
      * \sa m_max_used_consec_lcs
      */
    size_t m_curr_used_consec_lcs;

    /** \brief Number of consecutive loop closures to ignore after \b
      * m_max_used_consec_lcs have already been issued.
      *
      * \sa m_curr_ignored_consec_lcs, m_max_used_consec_lcs
      */
    size_t m_max_ignored_consec_lcs;

    /** \brief Consecutive Loop Closures that have currently been ignored
       *
       * \sa m_max_ignored_consec_lcs
       */
    size_t m_curr_ignored_consec_lcs;
    /** \}*/
};
}} // end namespaces

#endif // BASEGSO3D_H
