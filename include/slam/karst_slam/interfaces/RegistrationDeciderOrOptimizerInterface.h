#ifndef REGISTRATIONDECIDEROROPTIMIZERINTERFACE_H
#define REGISTRATIONDECIDEROROPTIMIZERINTERFACE_H

#include "karst_slam/BaseParams.h"

#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/utils/COutputLogger.h>
#include <memory>

// Forward declarations
namespace mrpt
{
    namespace obs
    {
        class CActionCollectionPtr;
        class CObservationPtr;
        class CObservation3DRangeScanPtr;
        class CObservationOdometryPtr;
    }

    namespace utils
    {
        class CConfigFileBase;
        class CConfigFile;
    }
}
namespace karst_slam
{
    namespace graph{class PoseGraph;}
    namespace obs
    {
        class ObservationMSIS_scanPtr;
        class ObservationDVLPtr;
        class ObservationMSISBeamPtr;
        class ObservationIMUWithUncertaintyPtr;
        class ObservationOdometryPtr;
    }
}

namespace karst_slam{namespace slam{
struct TModuleParams : public BaseParams
{
    virtual void loadFromConfigFile(
            const mrpt::utils::CConfigFileBase &source,
            const std::string &section);
    /**\brief Return a string with the configuration parameters
         */
    virtual void getAsString(std::string* params_out) const;
    int min_verbosity_level = 0;
};

/**
 * Class based on CRegistrationDeciderOrOptimizer.
 * We removed the elements related to the UI and specialized it directly on our own poseGraph class
 */
class RegistrationDeciderOptimizerInterface : public mrpt::utils::COutputLogger
{
public:
    explicit RegistrationDeciderOptimizerInterface(const std::string& name);
    RegistrationDeciderOptimizerInterface(const std::string& name, const std::string& configFile);
    RegistrationDeciderOptimizerInterface(const std::string& name, const mrpt::utils::CConfigFile& configFile);
    virtual ~RegistrationDeciderOptimizerInterface(){}

    /**\brief Generic method for fetching the incremental action-observations (or
     * observation-only) measurements
     *
     * \return True if operation was successful. Criteria for Success depend on
     * the decider/optimizer implementing this method
     */
    virtual bool updateState(const mrpt::obs::CActionCollectionPtr& action,
                             const mrpt::obs::CSensoryFramePtr& observations,
                             const mrpt::obs::CObservationPtr& observation,
                             mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr()) = 0;

    /**\brief Check if the current module (nrd, erd or gso) is correctly init
     */
    bool checkInit()const;

    /**\brief Load the necessary for the decider/optimizer configuration parameters.
     */
    void loadParams(const std::string& source_fname);
    virtual void loadParams(const mrpt::utils::CConfigFile& cfg);

    /**\brief Print the problem parameters - relevant to the decider/optimizer to the
     * screen in a unified/compact way.
     */
    virtual void printParams() const;
    /**\brief Fill the provided string with a detailed report of the
     * decider/optimizer state.
     *
     * Report should include (part of) the following:
     * - Timing of important methods
     * - Properties fo class at the current time
     * - Logging of commands until current time
     */
    virtual void getDescriptiveReport(std::string* report_str) const;

    /**\brief Fetch the graph on which the decider/optimizer will work on.
     *
     */
    virtual void setPoseGraphPtr(const std::shared_ptr<karst_slam::graph::PoseGraph>& graph);

    /**\brief Initialize the COutputLogger, CTimeLogger instances given the
     * name of the decider/optimizer at hand
     */
    virtual void initializeLoggers(const std::string& name);
    virtual void setClassName(const std::string& name);
    std::string getClassName() const { return m_class_name;}

protected:
    /**\brief Internal check to be implemented by derived classes. Called by checkInit()
     */
    virtual bool checkInit_()const = 0;

    virtual void updateStateFromAction(const mrpt::obs::CActionCollectionPtr &action){}
    virtual void updateStateFromObservation(const mrpt::obs::CObservationPtr& observation);
    virtual void updateStateFromObservations(const mrpt::obs::CSensoryFramePtr &observations);

    virtual void updateStateFromCObservation3DRangeScan(const mrpt::obs::CObservation3DRangeScanPtr& obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationMSIS_scan(const karst_slam::obs::ObservationMSIS_scanPtr& obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationAbsoluteOdometry(const mrpt::obs::CObservationOdometryPtr &obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationDVL(const karst_slam::obs::ObservationDVLPtr &obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationIMUWithUncertainty(const karst_slam::obs::ObservationIMUWithUncertaintyPtr &obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationMSISBeam(const karst_slam::obs::ObservationMSISBeamPtr &obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}
    virtual void updateStateFromObservationOdometry(const karst_slam::obs::ObservationOdometryPtr &obs){} //{MRPT_LOG_DEBUG_STREAM("Do nothing");}

    std::shared_ptr<karst_slam::graph::PoseGraph> m_pose_graph;

    /**\brief Name of the class instance */
    std::string m_class_name;

    TModuleParams m_genericParams;

    /**\brief Separator string to be used in debugging messages
     */
    static const std::string header_sep;
    static const std::string report_sep;
};
}} // end namespace
#endif // REGISTRATIONDECIDEROROPTIMIZERINTERFACE_H
