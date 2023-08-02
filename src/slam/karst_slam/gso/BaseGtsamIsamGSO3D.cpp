#include "karst_slam/gso/BaseGtsamIsamGSO3D.h"

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::gso;

BaseGtsamIsamGSO3D::BaseGtsamIsamGSO3D(const string& name) :
    BaseGtsamGSO3D<BaseGtsamIsamGSO3D>(name)
{}

BaseGtsamIsamGSO3D::BaseGtsamIsamGSO3D(const string& name,const CConfigFile& configFile) :
    BaseGtsamGSO3D<BaseGtsamIsamGSO3D>(name,configFile)
{}

BaseGtsamIsamGSO3D::BaseGtsamIsamGSO3D(const string& name,const string& configFile) :
    BaseGtsamGSO3D<BaseGtsamIsamGSO3D>(name,configFile)
{}

bool BaseGtsamIsamGSO3D::updateState(const mrpt::obs::CActionCollectionPtr& action,
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
        registered_new_node = true;
        m_last_total_num_of_nodes = nNode;
        if(opt_params.optimization_on_second_thread)
        {
            logFmt(LVL_DEBUG,"Join previous opt. thread ...");

            // Join the previous optimization thread
            mrpt::system::joinThread(m_thread_optimize);

            logFmt(LVL_DEBUG,"Previous opt. thread joined !");
            // Optimize the graph - run on a separate thread
            // Adapted here to use the optimizeGraph function of the inherited class
            m_thread_optimize = mrpt::system::createThreadFromObjectMethod(
                          /*obj = */ this,
                        /* func = */ &BaseGtsamIsamGSO3D::optimizeGraph);
            return true;
        }
        else// single threaded implementation
             _optimizeGraph();
    }
    registered_new_node = false;
    return false;
    MRPT_END;
}

void BaseGtsamIsamGSO3D::_optimizeGraph(bool is_full_update)
{
    MRPT_START;
    string funcStr = m_class_name + "::_optimizeGraph";

    CTicTac optimization_timer;
    optimization_timer.Tic();

    // Execute the optimization
    execOptimization();

    double elapsed_time = optimization_timer.Tac();
    logFmt(LVL_DEBUG,"Optimization of graph took: %fs", elapsed_time);

    MRPT_UNUSED_PARAM(elapsed_time);
    MRPT_END;
}

void BaseGtsamIsamGSO3D::optimizeGraph()
{
    MRPT_START;

    logFmt(LVL_DEBUG,
           "In optimizeGraph\n\tThreadID: %lu\n\tTrying to grab lock... ",
           mrpt::system::getCurrentThreadId());

    _optimizeGraph();

    logFmt(LVL_DEBUG, "2nd thread grabbed the lock..");
   MRPT_END;
}
