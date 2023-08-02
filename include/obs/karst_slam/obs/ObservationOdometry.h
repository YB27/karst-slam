#ifndef OBSERVATION_ODOMETRY_H
#define OBSERVATION_ODOMETRY_H

#include "karst_slam/typedefs.h"

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace karst_slam{namespace obs{

DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( ObservationOdometry, CObservation,OBS_IMPEXP )
class OBS_IMPEXP ObservationOdometry : public mrpt::obs::CObservation
{
    DEFINE_SERIALIZABLE( ObservationOdometry )
public:
    ObservationOdometry() = default;

    // Odometry of the robot -> express in its reference frame so equivalent to a "null" sensor pose
    inline void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const override
    {out_sensorPose = mrpt::poses::CPose3D();}
    inline void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override{}
    void getDescriptionAsText(std::ostream &o) const override;

    pose_pdf_t pose_pdf;
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( ObservationOdometry, mrpt::obs::CObservation, OBS_IMPEXP  )
}} // end namespaces


#endif // OBSERVATION_ODOMETRY_H
