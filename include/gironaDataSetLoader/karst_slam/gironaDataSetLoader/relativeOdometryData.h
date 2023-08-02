#ifndef DEF_RELATIVEODOMETRY_DATA_H
#define DEF_RELATIVEODOMETRY_DATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{
struct odometry_relative : data
{
    mrpt::obs::CObservationPtr convertToCObservation() override;

    void readFromLine_(std::stringstream& ss){}
    void dumpToConsole_(){}

    static Eigen::Matrix<double,6,6> odometry_cov_matrix;
    mrpt::poses::CPose3D currentPose;
    mrpt::poses::CPose3D prevPose;
};

}}

#endif // DEF_RELATIVEODOMETRY_DATA_H
