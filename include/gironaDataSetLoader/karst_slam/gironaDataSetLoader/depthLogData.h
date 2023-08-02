#ifndef DEF_DEPTHLOGDATA_H
#define DEF_DEPTHLOGDATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{
struct depthData : data
{
    void readFromLine_(std::stringstream &ss) override;

    mrpt::obs::CObservationPtr convertToCObservation() override;

    void dumpToConsole_() override;

    double depth;
};
}}

#endif // DEF_DEPTHLOGDATA_H
