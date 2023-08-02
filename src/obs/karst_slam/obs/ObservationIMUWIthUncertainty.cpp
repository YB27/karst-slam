#include "karst_slam/obs/ObservationIMUWithUncertainty.h"
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationIMUWithUncertainty, CObservation, karst_slam::obs)

void ObservationIMUWithUncertainty::writeToStream(CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        // Write the lower triangular part of the covariance matrices
        for(int i = 0; i < 3; i++)
        {
            for(int j = i ; j < 3; j++)
                out << m_staticCov(i,j);
        }

        for(int i = 0; i < 3; i++)
        {
            for(int j = i ; j < 3; j++)
                out << m_dynamicCov(i,j);
        }
        out << timestamp;
        out << sensorLabel;
        out << sensorPose;
        out << dataIsPresent;
        out << rawMeasurements;
    }
}

void ObservationIMUWithUncertainty::readFromStream(CStream &in, int version)
{
    // Read the lower triangular part of the covariance matrices
    for(int i = 0; i < 3; i++)
    {
        for(int j = i ; j < 3; j++)
            in >> m_staticCov(i,j);
    }

    for(int i = 0; i < 3; i++)
    {
        for(int j = i ; j < 3; j++)
            in >> m_dynamicCov(i,j);
    }

    in >> timestamp;
    in >> sensorLabel;
    in >> sensorPose;
    in >> dataIsPresent;
    if (dataIsPresent.size()<COUNT_IMU_DATA_FIELDS)
    {
        const size_t nOld = dataIsPresent.size();
        ASSERT_(rawMeasurements.size()==dataIsPresent.size());

        dataIsPresent.resize(COUNT_IMU_DATA_FIELDS);
        rawMeasurements.resize(COUNT_IMU_DATA_FIELDS);
        for (size_t i=nOld;i<COUNT_IMU_DATA_FIELDS;i++) {
            dataIsPresent[i]=false;
            rawMeasurements[i]=0;
        }
    }

    // Fill the lower part
    for(int j = 0; j < 3; j++)
    {
        for(int i = j ; i < 3; i++)
        {
            m_staticCov(i,j)  = m_staticCov(j,i);
            m_dynamicCov(i,j) = m_dynamicCov(j,i);
        }
    }
}

void ObservationIMUWithUncertainty::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Sensor pose on the robot: " << sensorPose << endl;

        o << mrpt::format("Orientation (degrees): (yaw,pitch,roll)=(%.06f, %.06f, %.06f)\n\n",
            RAD2DEG( rawMeasurements[IMU_YAW] ),
            RAD2DEG( rawMeasurements[IMU_PITCH] ),
            RAD2DEG( rawMeasurements[IMU_ROLL] ) );

        // Units:
        // Use "COUNT_IMU_DATA_FIELDS" so a compile error happens if the sizes don't fit ;-)
        static const char * imu_units[COUNT_IMU_DATA_FIELDS ] =
        {
            "m/s^2", //	IMU_X_ACC,
            "m/s^2", //	IMU_Y_ACC,
            "m/s^2", //	IMU_Z_ACC,
            "rad/s", //	IMU_YAW_VEL,
            "rad/s", //	IMU_PITCH_VEL,
            "rad/s", //	IMU_ROLL_VEL,
            "m/s", //	IMU_X_VEL,
            "m/s", //	IMU_Y_VEL,
            "m/s", //	IMU_Z_VEL,
            "rad", //	IMU_YAW,
            "rad", //	IMU_PITCH,
            "rad", //	IMU_ROLL,
            "m", //	IMU_X,
            "m", //	IMU_Y,
            "m",  //	IMU_Z
            "gauss", // IMU_MAG_X,
            "gauss", // IMU_MAG_Y,
            "gauss", // IMU_MAG_Z,
            "Pa", // IMU_PRESSURE,
            "m", // IMU_ALTITUDE,
            "deg.", // IMU_TEMPERATURE,
            "qx", // IMU_ORI_QUAT_X,
            "qy", // IMU_ORI_QUAT_Y,
            "qz", // IMU_ORI_QUAT_Z,
            "qw", // IMU_ORI_QUAT_W,
            "rad/s", //	IMU_YAW_VEL_GLOBAL
            "rad/s", //	IMU_PITCH_VEL_GLOBAL
            "rad/s", //	IMU_ROLL_VEL_GLOBAL
            "m/s^2", //	IMU_X_ACC_GLOBAL
            "m/s^2", //	IMU_Y_ACC_GLOBAL
            "m/s^2"  //	IMU_Z_ACC_GLOBAL
        };

    #define DUMP_IMU_DATA(x)  \
        o << mrpt::format("%15s = ",#x); \
        if (dataIsPresent[x]) \
        o << mrpt::format("%10f %s\n", rawMeasurements[x], imu_units[x]); \
        else  	o << "(not present)\n";

        DUMP_IMU_DATA(IMU_X_ACC)
        DUMP_IMU_DATA(IMU_Y_ACC)
        DUMP_IMU_DATA(IMU_Z_ACC)
        DUMP_IMU_DATA(IMU_YAW_VEL)
        DUMP_IMU_DATA(IMU_PITCH_VEL)
        DUMP_IMU_DATA(IMU_ROLL_VEL)
        DUMP_IMU_DATA(IMU_X_VEL)
        DUMP_IMU_DATA(IMU_Y_VEL)
        DUMP_IMU_DATA(IMU_Z_VEL)
        DUMP_IMU_DATA(IMU_YAW)
        DUMP_IMU_DATA(IMU_PITCH)
        DUMP_IMU_DATA(IMU_ROLL)
        DUMP_IMU_DATA(IMU_X)
        DUMP_IMU_DATA(IMU_Y)
        DUMP_IMU_DATA(IMU_Z)
        DUMP_IMU_DATA(IMU_MAG_X)
        DUMP_IMU_DATA(IMU_MAG_Y)
        DUMP_IMU_DATA(IMU_MAG_Z)
        DUMP_IMU_DATA(IMU_PRESSURE)
        DUMP_IMU_DATA(IMU_ALTITUDE)
        DUMP_IMU_DATA(IMU_TEMPERATURE)
        DUMP_IMU_DATA(IMU_ORI_QUAT_X)
        DUMP_IMU_DATA(IMU_ORI_QUAT_Y)
        DUMP_IMU_DATA(IMU_ORI_QUAT_Z)
        DUMP_IMU_DATA(IMU_ORI_QUAT_W)
        DUMP_IMU_DATA(IMU_YAW_VEL_GLOBAL)
        DUMP_IMU_DATA(IMU_PITCH_VEL_GLOBAL)
        DUMP_IMU_DATA(IMU_ROLL_VEL_GLOBAL)
        DUMP_IMU_DATA(IMU_X_ACC_GLOBAL)
        DUMP_IMU_DATA(IMU_Y_ACC_GLOBAL)
        DUMP_IMU_DATA(IMU_Z_ACC_GLOBAL)

    o << "Static orientation (yaw/roll/pitch) covariance matrix : \n";
    o << m_staticCov << endl;
    o << "Dynamic orientation (yaw/roll/pitch) covariance matrix : \n";
    o << m_dynamicCov << endl;
}
