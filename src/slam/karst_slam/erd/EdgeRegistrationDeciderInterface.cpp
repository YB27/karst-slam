#include "karst_slam/erd/EdgeRegistrationDeciderInterface.h"
#include "karst_slam/graph/PoseGraph.h"

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CObservation.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::slam;
using namespace karst_slam::graph;
using namespace karst_slam::erd;

//-----------------TERDParams-------------------//
//////////////////////////////////////////////////

void TERDparams::loadFromConfigFile(const CConfigFileBase &source, const string &section)
{
    MRPT_START
    LC_min_nodeid_diff = source.read_int(section, "LC_min_nodeid_diff", 4, false);
    
    nThetaSamples = source.read_int("MSIS", "Sonar_Micron_nThetaSamples", 200, true);
    pICP_max_distance = source.read_double("EdgeRegistrationDeciderParameters", "pICP_max_distance",1.,true);
    pythonScript_mpic = source.read_string("ICP", "pythonScript_mpic", "", true);
    MRPT_END
}

void TERDparams::getAsString(string* params_out) const
{
    MRPT_START
    *params_out += "------------------[ Edge Registration ]------------------\n";
    *params_out += mrpt::format("Minimum node id difference for Loop Closure : %ld\n", LC_min_nodeid_diff);
    MRPT_END
}


//----------------------------------------------//
//////////////////////////////////////////////////

EdgeRegistrationDeciderInterface::EdgeRegistrationDeciderInterface(const string& name,
                                                                   const CConfigFile& configFile):
        RegistrationDeciderOptimizerInterface(name, configFile),
        m_just_inserted_lc(false),
        m_override_registered_nodes_check(false),
        m_last_total_num_nodes(0),
        m_currNodeId(0),
        m_prevNodeId(0)
{}

EdgeRegistrationDeciderInterface::EdgeRegistrationDeciderInterface(const string& name,
                                                                   const string& configFile):
    EdgeRegistrationDeciderInterface(name, CConfigFile(configFile))
{}

EdgeRegistrationDeciderInterface::~EdgeRegistrationDeciderInterface() { }

void EdgeRegistrationDeciderInterface::loadParams(const CConfigFile &cfg)
{
    MRPT_START
    parent_t::loadParams(cfg);
    m_params.loadFromConfigFile(cfg, "EdgeRegistrationDeciderParameters");
    MRPT_END
}

void EdgeRegistrationDeciderInterface::printParams() const
{
    MRPT_START
    parent_t::printParams();
    m_params.dumpToConsole();
    MRPT_END
}

bool EdgeRegistrationDeciderInterface::updateState(const mrpt::obs::CActionCollectionPtr &action,
                                                   const mrpt::obs::CSensoryFramePtr &observations,
                                                   const mrpt::obs::CObservationPtr &observation,
                                                   mrpt::obs::CSensoryFramePtr generatedObservations)
{
    MRPT_START
    MRPT_UNUSED_PARAM(action);

    // check possible prior node registration
    checkIfNodeRegistered();

    if(observation.present())
        updateStateFromObservation(observation);
    else
        updateStateFromObservations(observations);

    checkRegistrationCondition();
    return true;
    MRPT_END
}

void EdgeRegistrationDeciderInterface::checkIfNodeRegistered()
{
    m_newNodeRegistered = false;
    size_t nNode = m_pose_graph->nodeCount();
    if(m_last_total_num_nodes < nNode)
    {
        m_prevNodeId = m_currNodeId;
        m_currNodeId = nNode - 1;
        m_newNodeRegistered = true;
        m_last_total_num_nodes = nNode;
        //logFmt(LVL_DEBUG, "New node has been registered!!!!");
    }
}

void EdgeRegistrationDeciderInterface::updateStateFromAction(const mrpt::obs::CActionCollectionPtr &action)
{
    logFmt(LVL_DEBUG,"Update state from action ...");
}

void EdgeRegistrationDeciderInterface::getDescriptiveReport(string* report_str) const
{
        stringstream ss("");
        parent_t::getDescriptiveReport(report_str);

        ss << "Edge Registration Decider Strategy [ERD]: " << endl;
        *report_str += ss.str();
}

void EdgeRegistrationDeciderInterface::registerNewEdge(const TNodeID& from,
                                                       const TNodeID& to,
                                                       const pose_pdf_t& rel_edge)
{
    MRPT_LOG_DEBUG_STREAM( "Registering new edge: " << from << " => "
                           << to << endl << "\tRelative Edge: " << rel_edge.getMeanVal().asString()
                           << "\tNorm: " << rel_edge.getMeanVal().norm());
    m_pose_graph->insertEdge(from,  to, rel_edge);
}
