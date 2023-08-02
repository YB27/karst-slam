#ifndef MSIS_NRD3D_H
#define MSIS_NRD3D_H

#include "NodeRegistrationDeciderInterface.h"

// Forward declarations
namespace karst_slam
{
    namespace sensors
    {
        class MSIS_primary;
        class MSIS_secondary;
    }
    namespace obs{class ObservationMSISBeamPtr;}
}

///* Node register class based on a MSIS (sonar).
///* Simply generate a new node at each full scan
namespace karst_slam{namespace nrd{

struct AccumOdometry_aux
{
    pose_pdf_t accumOdoSinceLastBeam;
    pose_pdf_t accumOdoSinceLastScanRefFrame;
    bool hasAccumulatedOdoSinceLastBeam = false;
};


class MSIS_NRD3D: public virtual NodeRegistrationDeciderInterface
{
public:

    // Public functions
    //////////////////////////////////////////////////////////////
    using InfMat = mrpt::math::CMatrixFixedNumeric<double,
                                                   pose_pdf_t::state_length,
                                                   pose_pdf_t::state_length>;
    /**\brief Typedef for accessing methods of the RangeScanRegistrationDecider
                 * parent class.
                 */
    using decider_t   = MSIS_NRD3D; /**< self type - Handy typedef */
    /**\brief Node Registration Decider */
    using parent_t    = NodeRegistrationDeciderInterface;
    /**\}*/

    /**\brief Class constructor */
    explicit MSIS_NRD3D(const std::string& configFile,
                        const std::string& sensorLabel);

    explicit MSIS_NRD3D(const mrpt::utils::CConfigFile& configFile,
                        const std::string& sensorLabel
                        /*const std::map<std::string, std::shared_ptr<karst_slam::sensors::MSIS>>& m_msises*/);
                        //const std::shared_ptr<karst_slam::sensors::MSIS>& msis = nullptr);

    /**\brief Class destructor */
    ~MSIS_NRD3D();

    bool checkInit_()const override;

    void setGeneratedObservations(mrpt::obs::CSensoryFramePtr generatedObservations) override;

    void setMSISPtr(const std::shared_ptr<karst_slam::sensors::MSIS_primary>& msis);
    //void setMSISesPtr(const std::map<std::string,std::shared_ptr<karst_slam::sensors::MSIS>>& msises);
    void setMSISPtr_noLoc(const std::shared_ptr<karst_slam::sensors::MSIS_secondary>& msis);

protected:
    bool checkRegistrationCondition() override;

    void updateAccumulatedOdometry_() override;
    void updateAccumulatedObservations_(bool register_node,
                                        const mrpt::obs::CSensoryFramePtr &observations,
                                        const mrpt::obs::CObservationPtr &observation,
                                        mrpt::obs::CSensoryFramePtr& generatedObservation) override;

    void updateStateFromObservationMSISBeam(const karst_slam::obs::ObservationMSISBeamPtr &obs) override;

    // Tmp accumulate odometry using given relative pose.
    // ToDo : receive DVL + IMU and compute odometry (here dead reckoning localization) with kalman filter
    void updateStateFromObservationOdometry(const karst_slam::obs::ObservationOdometryPtr &obs) override;

    bool registerNewNodeAtEnd(const pose_pdf_t& scan_refFrameGlobalPose,
                              const pose_pdf_t& scan_refFrameLocalPose,
                              const pose_pdf_t& scan_fromRefFrameToEndFramePose);

    // Fixme : common with CICPCriteriaERDMod
    //void setFixedCovariance(constraint_t& rel_edge);

    std::shared_ptr<karst_slam::sensors::MSIS_primary> m_msis;
    std::shared_ptr<karst_slam::sensors::MSIS_secondary> m_msisMap;
    std::map<std::string, AccumOdometry_aux> m_accumOdoForMSISes; ///< Accumulated Odometry for each MSIS scan

    //std::map<std::string, std::shared_ptr<karst_slam::sensors::MSIS>> m_msises;
    std::string m_sensorLabel;
    //std::vector<std::string> m_sensorsLabel;

    pose_pdf_t m_since_prev_beam_pdf;
    //pose_pdf_t m_prev_node_to_end_prev_scan;///< pose between the global reference frame of the previous scan (which correspond to a node) and the position where the prev scan ended
    bool m_hasAccumulatedOdoSinceLastBeam;
    /**\} */
};
}} // end namespaces

#endif // MSIS_NRD3D_H
