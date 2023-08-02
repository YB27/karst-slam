#ifndef OBSERVATION_DEPTH_H
#define OBSERVATION_DEPTH_H

#include "karst_slam/typedefs.h"

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>

namespace karst_slam{namespace obs{

DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( ObservationDepth, CObservation,OBS_IMPEXP )
class OBS_IMPEXP ObservationDepth : public mrpt::obs::CObservation
{
    DEFINE_SERIALIZABLE( ObservationDepth )
public:
    ObservationDepth() = default;

    double depth; //!< Observed depth value in meters
    double depth_std; //!< Standard deviation 

    // Odometry of the robot -> express in its reference frame so equivalent to a "null" sensor pose
    inline void getSensorPose( mrpt::poses::CPose3D &out_sensorPose ) const override{out_sensorPose = sensorPose;}
    inline void setSensorPose( const mrpt::poses::CPose3D &newSensorPose ) override{sensorPose = newSensorPose;}
    void getDescriptionAsText(std::ostream &o) const override;

     mrpt::poses::CPose3D sensorPose; //!< 6D pose of the sensor in the robot frame
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( ObservationDepth, mrpt::obs::CObservation, OBS_IMPEXP  )
}} // end namespaces

#endif // OBSERVATION_DEPTH_H
