#ifndef DEF_ODOMETRYLOGDATA_H
#define DEF_ODOMETRYLOGDATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{

/** Provided odometry */
// ToDo: Seems to be given by an EKF (IMU + DVL) ie dead reckoning estimation.
struct odometry : data
{
    void readFromLine_(std::stringstream& ss) override;

    mrpt::obs::CObservationPtr convertToCObservation() override;

    void dumpToConsole_() override;

    double t[3]; //!< position [x y z]
    double q[4]; //!< quaternion
};
}}

#endif // DEF_ODOMETRYLOGDATA_H
