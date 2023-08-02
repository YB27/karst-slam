#include "karst_slam/interfaces/RegistrationDeciderOrOptimizerInterface.h"
#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/obs/obs_includes.h"

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::slam;
using namespace karst_slam::graph;
using namespace karst_slam::obs;

const string RegistrationDeciderOptimizerInterface::header_sep = string(80, '-');
const string RegistrationDeciderOptimizerInterface::report_sep = string(2, '\n');

#define UPDATE_OBSERVATION_FROM_SENSORYFRAME(OBSERVATION_CLASS, SF) { \
int i = 0; \
OBSERVATION_CLASS##Ptr obsPtr = SF->getObservationByClass<OBSERVATION_CLASS>(i++); \
while(!obsPtr.null()) \
{ \
    updateStateFrom##OBSERVATION_CLASS(obsPtr); \
    obsPtr = observations->getObservationByClass<OBSERVATION_CLASS>(i++); \
}\
}

void TModuleParams::loadFromConfigFile(const CConfigFileBase &source, const string &section)
{
    MRPT_START
    min_verbosity_level = source.read_int(section, "min_verbosity_level", 0, false);
    MRPT_END
}

void TModuleParams::getAsString(string* params_out) const
{
    MRPT_START
    *params_out += "------------------[ Generic Module Params ]------------------\n";
    *params_out += mrpt::format("Minimum level of verbosity : %d\n", min_verbosity_level);
    MRPT_END
}

RegistrationDeciderOptimizerInterface::RegistrationDeciderOptimizerInterface(const string& name) :
    mrpt::utils::COutputLogger(name),
    m_pose_graph(nullptr),
    m_class_name(name)
{
    setMinLoggingLevel(LVL_DEBUG);
}

RegistrationDeciderOptimizerInterface::RegistrationDeciderOptimizerInterface(const string& name,
                                                                             const mrpt::utils::CConfigFile& configFile):
    RegistrationDeciderOptimizerInterface(name)
{
    MRPT_START
    initializeLoggers(m_class_name);
    loadParams(configFile);
    MRPT_END
}

RegistrationDeciderOptimizerInterface::RegistrationDeciderOptimizerInterface(const string& name,
                                                                             const string& configFile) :
    RegistrationDeciderOptimizerInterface(name, CConfigFile(configFile))
{}

bool RegistrationDeciderOptimizerInterface::checkInit() const
{
    if(m_pose_graph == nullptr)
    {
        logFmt(LVL_ERROR,"Invalid pose graph pointer. Have you called setPoseGraphPtr() ?");
        return false;
    }
    return checkInit_();
}

void RegistrationDeciderOptimizerInterface::updateStateFromObservations(const CSensoryFramePtr &observations)
{
    MRPT_START
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(CObservation3DRangeScan, observations)
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(ObservationMSIS_scan, observations)
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(ObservationDVL, observations)
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(ObservationIMUWithUncertainty, observations)
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(ObservationMSISBeam, observations)
    UPDATE_OBSERVATION_FROM_SENSORYFRAME(ObservationOdometry, observations)
    // Add here other CObservation classes

    MRPT_END
}

void RegistrationDeciderOptimizerInterface::updateStateFromObservation(const CObservationPtr &observation)
{
    MRPT_START
    if(IS_CLASS(observation, CObservation3DRangeScan))
          updateStateFromCObservation3DRangeScan(static_cast<CObservation3DRangeScanPtr>(observation));
    else if(IS_CLASS(observation, ObservationMSIS_scan))
          updateStateFromObservationMSIS_scan(static_cast<ObservationMSIS_scanPtr>(observation));
    else if(IS_CLASS(observation, ObservationMSISBeam))
          updateStateFromObservationMSISBeam(static_cast<ObservationMSISBeamPtr>(observation));
    else if(IS_CLASS(observation, ObservationDVL))
          updateStateFromObservationDVL(static_cast<ObservationDVLPtr>(observation));
    else if(IS_CLASS(observation, ObservationIMUWithUncertainty))
         updateStateFromObservationIMUWithUncertainty(static_cast<ObservationIMUWithUncertaintyPtr>(observation));
    else if(IS_CLASS(observation, ObservationOdometry))
         updateStateFromObservationOdometry(static_cast<ObservationOdometryPtr>(observation));
    //{...}

    MRPT_END
}

void RegistrationDeciderOptimizerInterface::initializeLoggers(const string& name)
{
    using namespace mrpt::utils;

    setClassName(name); // all the names in one call
    logging_enable_keep_record = true;

    // just for the messages until reading the actual verbosity level, set it to debug.
    setMinLoggingLevel(LVL_DEBUG);
    MRPT_LOG_DEBUG_STREAM("Initialized time, output logger instances." << endl);
}

void RegistrationDeciderOptimizerInterface::setClassName(const string& name)
{
    this->m_class_name = name;
    this->setLoggerName(this->m_class_name);
}

void RegistrationDeciderOptimizerInterface::loadParams(const string& source_fname)
{
    MRPT_LOG_DEBUG_STREAM("Loading corresponding parameters");
    CConfigFile cfg(source_fname);
    loadParams(cfg);
}

void RegistrationDeciderOptimizerInterface::loadParams(const CConfigFile& cfg)
{
    MRPT_LOG_DEBUG_STREAM("Loading corresponding parameters");
    m_genericParams.loadFromConfigFile(cfg,"GeneralConfiguration");
    setMinLoggingLevel(VerbosityLevel(m_genericParams.min_verbosity_level));
}

void RegistrationDeciderOptimizerInterface::printParams() const
{
    MRPT_LOG_DEBUG_STREAM("Printing corresponding parameters");
    m_genericParams.dumpToConsole();
}

void RegistrationDeciderOptimizerInterface::getDescriptiveReport(string* report_str) const
{
    MRPT_LOG_DEBUG_STREAM("Generating corresponding report");
    // TODO - give the compact form here!
}

void RegistrationDeciderOptimizerInterface::setPoseGraphPtr(const shared_ptr<PoseGraph>& graph)
{
    // Pose graph should be initialized with a root node at (0,0,0,0,0,0) pose
    ASSERT_(m_pose_graph->getRoot() == 0 && m_pose_graph->getNodes().size() == 1)
    //MRPT_LOG_DEBUG_STREAM("Fetched the graph pointer");
    m_pose_graph = graph;
}
