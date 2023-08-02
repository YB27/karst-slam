#ifndef DEF_SONARBEAMLOGDATA_H
#define DEF_SONARBEAMLOGDATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{

struct sonar_beam : data
{
    void readFromLine_(std::stringstream& ss) override;

    mrpt::obs::CObservationPtr convertToCObservation() override;

    void dumpToConsole_() override;

    static int sonar_angular_nstep;
    static double sonar_angular_step;
    double angle; //!< Expressed in rad
    int step_idx; //!< Corresponding step idx of the angle
    int nbins; //<! Number of intensity bins
    int max_range; //!< Sonar max range (in meters)
    std::vector<int> intensities; //!< Vector of intensities (size nbins)
};
}}

#endif // DEF_SONARBEAMLOGDATA_H
