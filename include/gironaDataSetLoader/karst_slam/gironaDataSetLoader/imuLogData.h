#ifndef DEF_IMULOGDATA_H
#define DEF_IMULOGDATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{

struct imuData : data
{
    void readFromLine_(std::stringstream &ss) override;

    mrpt::obs::CObservationPtr convertToCObservation() override;

    void dumpToConsole_() override;

    double roll;
    double pitch;
    double yaw;
    double q[4]; //!< quaternion : [qx qy qz qw] in logs. Take care if used with mrpt (first qw)
    double temp;
    double m[3]; //!< magneto
    double a[3]; //!< accelero
    double g[3]; //!< gyro
    double b[3]; //!< Seems to be the estimated gyro biases (from kalman filter ?)
};
}}

#endif // DEF_IMULOGDATA_H
