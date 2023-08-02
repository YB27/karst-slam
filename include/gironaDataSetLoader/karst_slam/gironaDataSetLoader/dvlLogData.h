#ifndef DEF_DVLLOGDATA_H
#define DEF_DVLLOGDATA_H

#include "baseLogData.h"

namespace karst_slam{ namespace UW_caveDataSet{
struct dvlData : data
{
    void readFromLine_(std::stringstream &ss) override;

    mrpt::obs::CObservationPtr convertToCObservation() override;

    void dumpToConsole_() override;

    bool dataGood [4]; //!< I suppose that it is the goodness of each beam measurement (there are 4 beams)
    double altitudeBeam [4];
    double bottomVelocityBeam [4];
    double waterVelocityBeam [4];
    int waterVelocityCredit [4];
    double velocityInst[3]; //!< Velocities along each axis but meaning of Inst ???? ToDo : To check
    int velocityInstFlag; //!< Meaning ??? ToDo : To check
    double velocityEarth[3]; //!< Velocities along each axis but meaning of Inst ???? ToDo : To check
    int velocityEarthFlag; //!< Meaning ?? ToDo : To check
    double waterVelocityInst [3];
    int waterVelocityInstFlag;
    double waterVelocityEarth [3];
    int waterVelocityEarthFlag;
    double temperature;
    double pressure;
    double salinity;
};
}}

#endif // DEF_DVLLOGDATA_H
