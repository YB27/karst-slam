#ifndef OBSERVATIONDVL_H
#define OBSERVATIONDVL_H

#include "karst_slam/typedefs.h"
#include "karst_slam/mrpt_poses_frwd.h"

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>

namespace karst_slam{ namespace obs {
DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( ObservationDVL, CObservation,OBS_IMPEXP )
class OBS_IMPEXP ObservationDVL : public mrpt::obs::CObservation
{
    DEFINE_SERIALIZABLE( ObservationDVL )

public:
    ObservationDVL() = default;
    ~ObservationDVL(){}

    void getDescriptionAsText(std::ostream& o) const override;

    void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override {out_sensorPose = m_sensorPose;}
    void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override {m_sensorPose = newSensorPose;}
    inline void getVelocities(double& vx, double& vy, double& vz) const {vx = m_vx; vy = m_vy; vz = m_vz;}

protected:
    inline double getAccuracy(const double& measure)const {return m_relativeVelocityAccuracy*measure + m_absoluteVelocityAccuracy;}

    double m_relativeVelocityAccuracy; //!< Final accuracy is %relative +- absolute (eg 0.2% +- 1mm/s)
    double m_absoluteVelocityAccuracy;

    double m_vx; //!< Measured velocity along the x axis
    double m_vy; //!< Measured velocity along the y axis
    double m_vz; //!< Measured velocity along the z axis

    mrpt::poses::CPose3D m_sensorPose; //!< 6D pose of the sensor in the robot frame
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( ObservationDVL, mrpt::obs::CObservation,OBS_IMPEXP )
}} // end namespaces

#endif // OBSERVATIONDVL_H
