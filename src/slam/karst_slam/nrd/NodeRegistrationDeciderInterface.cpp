#include "karst_slam/nrd/NodeRegistrationDeciderInterface.h"
#include "karst_slam/graph/PoseGraph.h"

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CActionCollection.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace karst_slam;
using namespace karst_slam::nrd;
using namespace karst_slam::graph;

//-----------------TNRDParams-------------------//
//////////////////////////////////////////////////

void TNRDparams::loadFromConfigFile(const CConfigFileBase &source, const string &section)
{
    MRPT_START;
    cout << "TNRDparams::loadFromConfigFile start " << endl;
    registration_max_distance = source.read_double( section,
                    "registration_max_distance",
                    0.5 /* meter */, false);
    registration_max_angle = source.read_double( section,
                    "registration_max_angle",
                    60 /* degrees */, false);
    registration_max_angle = DEG2RAD(registration_max_angle);
    use_distance_node_reg = source.read_bool(section, "use_distance_node_reg", true, false);
    use_angle_difference_node_reg = source.read_bool(section, "use_angle_difference_node_reg", true, false);

    MRPT_END;
}

void TNRDparams::getAsString(string *params_out) const
{
    MRPT_START;

    double max_angle_deg = RAD2DEG(registration_max_angle);
    params_out->clear();

    *params_out +=
            "------------------[ Node Registration ]------------------\n";
    *params_out += mrpt::format("Used distance as registration criteria ? %d\n", use_distance_node_reg);
    if(use_distance_node_reg)
    {
        *params_out += mrpt::format(
                        "Max distance for registration = %.2f m\n", registration_max_distance);
    }
    *params_out += mrpt::format("Used angle as registration criteria ? %d\n", use_angle_difference_node_reg);
    if(use_angle_difference_node_reg)
    {
        *params_out += mrpt::format(
                        "Max angle for registration    = %.2f deg\n", registration_max_angle);
    }

    MRPT_END;
}

//----------------------------------------------//
//////////////////////////////////////////////////

NodeRegistrationDeciderInterface::NodeRegistrationDeciderInterface(const string& name,
                                                                   const CConfigFile& configFile):
    RegistrationDeciderOptimizerInterface(name, configFile),
    m_prev_registered_nodeID(INVALID_NODEID)
{
    using namespace mrpt::poses;

    m_init_inf_mat.unit();
    m_init_inf_mat *= 10000;
    resetPDF(&m_since_prev_node_PDF);
    resetPDF(&m_last_odometry_increment);
}

NodeRegistrationDeciderInterface::NodeRegistrationDeciderInterface(const string& name,
                                                                   const string& configFile):
    NodeRegistrationDeciderInterface(name, CConfigFile(configFile))
{}

NodeRegistrationDeciderInterface::~NodeRegistrationDeciderInterface()
{}


void NodeRegistrationDeciderInterface::getDescriptiveReport(string* report_str) const
{
    MRPT_START;

    stringstream ss("");
    parent_t::getDescriptiveReport(report_str);

    ss << "Node Registration Decider Strategy [NRD]: " << endl;
    *report_str += ss.str();

    MRPT_END;
}

void NodeRegistrationDeciderInterface::setPoseGraphPtr(const std::shared_ptr<karst_slam::graph::PoseGraph>& graph)
{
    parent_t::setPoseGraphPtr(graph);
    m_prev_registered_nodeID = graph->getRoot(); // should be zero
}


bool NodeRegistrationDeciderInterface::checkRegistrationCondition(){return false;}

bool NodeRegistrationDeciderInterface::registerNewNodeAtEnd(const pose_pdf_t& constraint)
{
    MRPT_START;

    // Add the new pose.
    {
        pose_pdf_t tmp_pose = getCurrentRobotPosEstimation();
        m_pose_graph->insertNode(/*to,*/ tmp_pose,constraint);

        // Normally, edge are inserted as constraint measure by sensor observations (see ERD)
        // For the first edge between the root node and the first added node, edge constraint can only be the same as odometry
        TNodeID newNodeID = m_pose_graph->getLastNodeID();
        if(m_prev_registered_nodeID == m_pose_graph->getRoot())
            m_pose_graph->insertEdgeAtEnd(m_prev_registered_nodeID,newNodeID,constraint);//this->m_graph->insertEdgeAtEnd(from, to, constraint);
         m_prev_registered_nodeID = newNodeID;
    }

    return true;
    MRPT_END;
}

bool NodeRegistrationDeciderInterface::registerNewNodeAtEnd()
{
    bool res = registerNewNodeAtEnd(this->m_since_prev_node_PDF);

    // reset the PDF since the last registered node position
    this->resetPDF(&m_since_prev_node_PDF);

    return res;
}

void NodeRegistrationDeciderInterface::resetPDF(pose_pdf_t* c)
{
    MRPT_START;
    ASSERT_(c);

    *c = pose_pdf_t();
    ASSERT_(c->isInfType());
    c->cov_inv = m_init_inf_mat;

    MRPT_END;
}

pose_pdf_t NodeRegistrationDeciderInterface::getCurrentRobotPosEstimation() const
{
    pose_pdf_t pose_out;

    // For the first node added (excluding the root), the last node pdf is invalid (null information matrix).
    // So don't use it as the node pose is in this case simply the accumulated odometry
    if (m_prev_registered_nodeID > 0) {
        pose_out = m_pose_graph->getLastNodeEstimatedPosePdf(); //m_pose_graph->getLastNodePosePdf();
        //cout << "m_pose_graph->getLastNodePosePdf() : " << pose_out.mean << endl;
        //cout << "m_since_prev_node_PDF in getcurrent : " << m_since_prev_node_PDF << endl;
        pose_out += m_since_prev_node_PDF;
        //cout << "pose_out : " << pose_out << endl;
        // pose_out += m_since_prev_node_PDF;
        //cout << "sincePrevNodePdf : " << m_since_prev_node_PDF << endl;
        //pose_out = m_pose_graph->getNodePose(m_prev_registered_nodeID);// this->m_graph->nodes.at(m_prev_registered_nodeID);
    }
    else
        pose_out = m_since_prev_node_PDF;

    //cout << "since_prev_node_pdf : " << m_since_prev_node_PDF << endl;
    return pose_out;
}

bool NodeRegistrationDeciderInterface::updateState(const CActionCollectionPtr &action,
                                                   const CSensoryFramePtr &observations,
                                                   const CObservationPtr &observation,
                                                   CSensoryFramePtr generatedObservations)
{
    MRPT_START
    m_hasReceivedOdometry = false;
    if(observation)
        updateStateFromObservation(observation);
    else
    {
        // ToDo : Should consider IMU/DVL (or other dead reckoning obs) as "action"
        // ie should be treated first before real observations.
        updateStateFromAction(action);
        updateStateFromObservations(observations);
    }

//    cout << "current odo only poses : " << endl;
//    cout << m_curr_odometry_only_pose << endl;
//    cout << "current robot estimation before registration : " << endl;
//    cout << getCurrentRobotPosEstimation() << endl;

    // Update the accumulated poses since the last added node
    updateAccumulatedOdometry(/*register_node*/);

    bool register_node  = checkRegistrationCondition();
//    cout << "current robot estimation after registration : " << endl;
//    cout << getCurrentRobotPosEstimation() << endl;
    updateAccumulatedObservations(register_node, observations, observation, generatedObservations);

    return register_node;
    MRPT_END
}

void NodeRegistrationDeciderInterface::updateAccumulatedObservations(bool register_node,
                                                                      const CSensoryFramePtr &observations,
                                                                      const CObservationPtr &observation,
                                                                      CSensoryFramePtr &generatedObservation)
{
    updateAccumulatedObservations_(register_node, observations, observation,generatedObservation);
    if(register_node)
        m_observationsOfLastRegistration = move(m_accumulatedObservationsSinceLastRegistration);
}

void NodeRegistrationDeciderInterface::updateStateFromAction(const CActionCollectionPtr &action)
{
    MRPT_START
    // Get the odometry increment and update odometry-only poses
    mrpt::poses::CPose3DPDFGaussian increment_pdf;
    if(action->getFirstMovementEstimation(increment_pdf))
    {
        // update the relative PDF of the path since the LAST node was inserted
        m_last_odometry_increment.copyFrom(increment_pdf);
        //m_curr_odometry_only_pose += m_last_odometry_increment.mean();
        m_hasReceivedOdometry = true;
    }
    MRPT_END
}

void NodeRegistrationDeciderInterface::updateAccumulatedOdometry(/*bool isRegisterNode*/)
{
    if(m_hasReceivedOdometry)
    {
        //cout << "[NodeRegistrationDeciderInterface::updateAccumulatedOdometry] m_since_prev_node_PDF = " << m_since_prev_node_PDF.mean << endl;
        m_since_prev_node_PDF += m_last_odometry_increment;

        // Update estimated pose if only odometry information was used
        m_curr_odometry_only_pose += m_last_odometry_increment.getMeanVal();

        // Derived-specific updates
        updateAccumulatedOdometry_();

        // Reset the last odometry
        resetPDF(&m_last_odometry_increment);

        //MRPT_LOG_DEBUG_STREAM("Accumulated odometry since last node : " << m_since_prev_node_PDF << endl);
    }
}

void NodeRegistrationDeciderInterface::loadParams(const CConfigFile& cfg)
{
    parent_t::loadParams(cfg);
    m_params.loadFromConfigFile(cfg, "NodeRegistrationDeciderParameters");
}

void NodeRegistrationDeciderInterface::printParams() const
{
    MRPT_START
    parent_t::printParams();
    m_params.dumpToConsole();
    MRPT_END
}
