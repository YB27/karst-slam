#ifndef DEF_BASELOGDATA_H
#define DEF_BASELOGDATA_H

#include "karst_slam/obs/obs_includes.h"
#include <iostream>

namespace karst_slam{ namespace UW_caveDataSet{

/** Enums for the sensor data types found in the Girona dataset */
enum DATA_FROM_SENSOR{DEPTH = 0, //!< Data from depth (pression) sensor
                      DVL = 1, //!< Data from the DVL (Doppler Velocity Logs)
                      IMU_ADIS = 2, //!< Data from the ADIS IMU (Inertial Measurement Unit)
                      IMU_XSENS = 3, //!< Data from the XSENS IMU (Inertial Measurement Unit)
                      /** Odometries (relative displacement computed based on absolute odometry data).
                       *  Note that it is not in the dataset nut compute from it */
                      ODOMETRY = 4, 
                      ODOMETRY_ABSOLUTE = 5, //!< Absolute pose of the robot (obtained with deadreckoning ie kalman IMU+DVL)
                      SONAR_MICRON = 6, //!< Data from the Micron sonar (horizontal sonar)
                      SONAR_SEAKING = 7}; //!< Data from the Seaking sonar (vertical sonar)

struct data
{
    /** Read data from a line of the dataset file */
    void readFromLine(std::stringstream& ss,
                      DATA_FROM_SENSOR d);

    /** Convert data to a CObservation type (for compatibility with MRPT) */
    virtual mrpt::obs::CObservationPtr convertToCObservation() = 0;

    /** Dump the data to console */
    void dumpToConsole();

    /** Extract data from a single line */
    virtual void readFromLine_(std::stringstream& ss) = 0;
    
    virtual void dumpToConsole_() = 0;

    std::string stamp;
    DATA_FROM_SENSOR dataType = (DATA_FROM_SENSOR)-1;
};
}}

#endif // DEF_BASELOGDATA_H
