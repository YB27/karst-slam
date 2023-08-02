#ifndef EDGEREGISTRATIONDECIDERINTERFACE_H
#define EDGEREGISTRATIONDECIDERINTERFACE_H

#include "karst_slam/typedefs.h"
#include "karst_slam/interfaces/RegistrationDeciderOrOptimizerInterface.h"
#include "karst_slam/BaseParams.h"
#include "karst_slam/sensors/MSIS.h"


namespace karst_slam{ namespace erd{

struct TERDparams : public BaseParams
{
    virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,
                                    const std::string &section);
    /**\brief Return a string with the configuration parameters
         */
    virtual void getAsString(std::string* params_out) const;

    size_t LC_min_nodeid_diff;
    karst_slam::sensors::MSISParams verticalSonarParams;
    karst_slam::sensors::MSISParams horizontalSonarParams;
    int nThetaSamples;
    double pICP_max_distance;
    std::string pythonScript_mpic;

    bool has_read_config;
};

/** Class based on MRPT CEdgeRegistrationDecider with slight adaptation */
class EdgeRegistrationDeciderInterface : public karst_slam::slam::RegistrationDeciderOptimizerInterface
{
public:
    /**\brief Handy typedefs */
    /**\{*/
    /**\brief Parent of current class */
    using parent_t = RegistrationDeciderOptimizerInterface;
    /**\}*/

    /**\brief Default class constructor.*/
    EdgeRegistrationDeciderInterface(const std::string& name, const std::string& configFile);
    EdgeRegistrationDeciderInterface(const std::string& name, const mrpt::utils::CConfigFile& configFile);

    /**\brief Default class destructor.*/
    virtual ~EdgeRegistrationDeciderInterface();
    /**\brief Generic method for fetching the incremental action/observation
     * readings from the calling function.
     *
     * Implementations of this interface should use (part of) the specified
     * parameters and call the checkRegistrationCondition to check for
     * potential Edge registration
     */
    virtual bool updateState(const mrpt::obs::CActionCollectionPtr& action,
                             const mrpt::obs::CSensoryFramePtr& observations,
                             const mrpt::obs::CObservationPtr& observation,
                             mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr());

    /**\brief Used by the caller to query for possible loop closures in the
* last edge registration procedure.
*/
    virtual bool justInsertedLoopClosure() const {return m_just_inserted_lc;}

    virtual void getDescriptiveReport(std::string* report_str) const;

    virtual void loadParams(const mrpt::utils::CConfigFile& cfg);
    virtual void printParams() const;

    inline void setMSISparams(const karst_slam::sensors::MSISParams& verticalParams,
                              const karst_slam::sensors::MSISParams& horizontalParams) 
    {m_params.verticalSonarParams = verticalParams; 
     m_params.horizontalSonarParams = horizontalParams;}

protected:
    virtual void updateStateFromAction(const mrpt::obs::CActionCollectionPtr &action);

    virtual void checkIfNodeRegistered() final;

    /**\name Registration criteria checks
     *\brief Check whether a new edge should be registered in the
     * graph.
     *
     * If condition(s) for edge registration is satisfied, method should call
     * the registerNewEdge method.
     */
    /**\{*/
    virtual void checkRegistrationCondition() =0; //(const std::set<mrpt::utils::TNodeID>&) = 0;

    /**\}*/
    /**\brief Register a new constraint/edge in the current graph.
     *
     * Implementations of this class should provide a wrapper around
     * GRAPH_T::insertEdge method.
*/
    virtual void registerNewEdge( const mrpt::utils::TNodeID& from,
                                  const mrpt::utils::TNodeID& to,
                                  const pose_pdf_t& rel_edge);

    TERDparams m_params;

    bool m_just_inserted_lc;
    /**\brief Indicates whether the ERD implementation expects, at most one
     * single node to be registered, between successive calls to the
     * updateState method.
     *
     * By default set to false.
     */
    bool m_override_registered_nodes_check;

    /**\brief Keep track of the total number of registered nodes since the last
     * time class method was called
     */
    size_t m_last_total_num_nodes;

    bool m_newNodeRegistered;///< True if new node(s) has (have) been registered since the last call to this class update
    size_t m_currNodeId;
    size_t m_prevNodeId;
};
}} // end namespaces

#endif // EDGEREGISTRATIONDECIDERINTERFACE_H
