#ifndef NODEREGISTRATIONDECIDERINTERFACE_H
#define NODEREGISTRATIONDECIDERINTERFACE_H

#include "karst_slam/interfaces/RegistrationDeciderOrOptimizerInterface.h"
#include "karst_slam/BaseParams.h"
#include "karst_slam/typedefs.h"

namespace karst_slam{namespace nrd{
struct TNRDparams : public BaseParams
{
    virtual void loadFromConfigFile(
            const mrpt::utils::CConfigFileBase &source,
            const std::string &section);
    /**\brief Return a string with the configuration parameters
         */
    virtual void getAsString(std::string* params_out) const;

    // max values for new node registration
    double registration_max_distance;
    double registration_max_angle;

    // Which criteria to be used (angle/distance)
    bool use_angle_difference_node_reg;
    bool use_distance_node_reg;
};

/**
 * Class based on CNodeRegistrationDecider
 */
class NodeRegistrationDeciderInterface : public karst_slam::slam::RegistrationDeciderOptimizerInterface
{
public:
    using parent_t  = RegistrationDeciderOptimizerInterface;
    using inf_mat_t = mrpt::math::CMatrixFixedNumeric<double,
                                                      pose_pdf_t::state_length,
                                                      pose_pdf_t::state_length>;

    /**\brief Default class constructor.*/
    NodeRegistrationDeciderInterface(const std::string& name, const std::string& configFile);
    NodeRegistrationDeciderInterface(const std::string& name, const mrpt::utils::CConfigFile& configFile);

    /**\brief Default class destructor.*/
    virtual ~NodeRegistrationDeciderInterface();

    /**\brief Getter method for fetching the currently estimated robot position.
     *
     * In single-robot situations this is most likely going to be the last
     * registered node position + an position/uncertainty increment from that
     * position
     */
    virtual pose_pdf_t getCurrentRobotPosEstimation() const;

    /**
     * @brief getAccumulatedObservations
     * @return The internal accumulated observations since the previous node was registered.
     */
    inline const std::vector<mrpt::obs::CObservationPtr>& getObservationsLastRegistration()const {return m_observationsOfLastRegistration;}

    inline const pose_t& getCurrOdometryOnlyPose() const {return m_curr_odometry_only_pose;}

    /**\brief Generic method for fetching the incremental action-observations
     * (or observation-only) depending on the rawlog format readings from the
     * calling function.
     *
     * Implementations of this interface should use (part of) the specified
     * parameters and call the checkRegistrationCondition to check for
     * potential node registration
     *
     * \return True upon successful node registration in the graph
     */
    bool updateState(const mrpt::obs::CActionCollectionPtr &action,
                     const mrpt::obs::CSensoryFramePtr &observations,
                     const mrpt::obs::CObservationPtr &observation,
                     mrpt::obs::CSensoryFramePtr generatedObservations = mrpt::obs::CSensoryFramePtr());

    virtual void setGeneratedObservations(mrpt::obs::CSensoryFramePtr generatedObservations){}

    /**\brief Fetch the graph on which the decider/optimizer will work on.
     *
     */
    void setPoseGraphPtr(const std::shared_ptr<karst_slam::graph::PoseGraph>& graph) override;

    virtual void getDescriptiveReport(std::string* report_str) const;


    virtual void loadParams(const mrpt::utils::CConfigFile& cfg);
    virtual void printParams() const;

    inline pose_t getCurrentOdometryOnlyPose(){return m_curr_odometry_only_pose;}
    inline mrpt::utils::TNodeID getLastRegisteredNodeId() const {return m_prev_registered_nodeID;}

protected:
    void updateAccumulatedObservations(bool register_node,
                                       const mrpt::obs::CSensoryFramePtr &observations,
                                       const mrpt::obs::CObservationPtr &observation,
                                       mrpt::obs::CSensoryFramePtr& generatedObservation);
    virtual void updateAccumulatedObservations_(bool register_node,
                                                const mrpt::obs::CSensoryFramePtr &observations,
                                                const mrpt::obs::CObservationPtr &observation,
                                                mrpt::obs::CSensoryFramePtr& generatedObservation){}
    virtual void updateAccumulatedOdometry();
    virtual void updateAccumulatedOdometry_(){}

    virtual void updateStateFromAction(const mrpt::obs::CActionCollectionPtr &action);
//    virtual void updateStateFromObservation(const mrpt::obs::CObservationPtr& observation);
//    virtual void updateStateFromObservations(const mrpt::obs::CSensoryFramePtr &observations);

//    virtual void updateStateFromObservation3DRangeScan(const mrpt::obs::CObservation3DRangeScanPtr& obs) override;

    /**\brief Reset the given PDF method and assign a fixed high-certainty
     * Covariance/Information matrix
     */
    void resetPDF(pose_pdf_t* c);
    /**\brief Check whether a new node should be registered in the
     * graph.
     *
     * This should be the key-method in any implementation of this
     * interface. Should call registerNewNodeAtEnd method if the registration
     * condition is satisfied.
     *
     * \return True upon successful node registration in the graph
     */
    virtual bool checkRegistrationCondition();
    /** Add a new constraint at the end of the graph.
     * \param[in] constraint Constraint transformation from the latest
     * registered to the new node.
     *
     * \return True upon successful node registration.
     */
    bool registerNewNodeAtEnd(const pose_pdf_t& constraint);
    /**\brief Same goal as the previous method - uses the m_since_prev_node_PDF
     * as the constraint at the end.
     */
    bool registerNewNodeAtEnd();

    /**\brief Store the last registered NodeID.
     *
     * We don't store its pose since it will most likely change due to calls to the
     * graph-optimization procedure / dijkstra_node_estimation
     */
    mrpt::utils::TNodeID m_prev_registered_nodeID;
    /**\brief Tracking the PDF of the current position of the robot with
     * regards to the <b previous registered node</b>.
     */
    pose_pdf_t m_since_prev_node_PDF;

    bool m_hasReceivedOdometry;
    pose_pdf_t m_last_odometry_increment;

    // Absolute odometry given by GPS or the detection of an anchor with a known global pose etc..
    //pose_pdf_t m_last_absolute_odometry;
    //pose_pdf_t m_since_prev_absolute_odometry;

    /**\brief Initial information matrix for paths
     *
     * Large values for this indicate that I am sure of the corresponding
     * (initial) pose
     */
    inf_mat_t m_init_inf_mat;
    /**\brief pose_t estimation using only odometry information.
     * \note Utilized only in observation-only rawlogs.
     *
     */
    pose_t m_curr_odometry_only_pose;
    /**\brief pose_t estimation using only odometry information.
     * \note Utilized only in observation-only rawlogs.
     *
     * Resets next time an ICP edge/Odometry measurement is utilized for
     * updating the estimated robot position.
     */
    pose_t m_last_odometry_only_pose;

    std::vector<mrpt::obs::CObservationPtr> m_observationsOfLastRegistration;
    std::vector<mrpt::obs::CObservationPtr> m_accumulatedObservationsSinceLastRegistration;

    TNRDparams m_params;
};
}} // end namespaces
#endif //NODEREGISTRATIONDECIDERINTERFACE_H
