#include "karst_slam/gironaDataSetLoader/imuLogData.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

void imuData::readFromLine_(stringstream &ss)
{
    string valStr;
    // Roll
    getline(ss,valStr, ',');
    roll  = stod(valStr);

    // Pitch
    getline(ss,valStr, ',');
    pitch = stod(valStr);

    // Yaw
    getline(ss,valStr, ',');
    yaw   = stod(valStr);

    // Quaternion
    for(int i = 0; i < 4 ; i++)
    {
        getline(ss,valStr, ',');
        q[i] = stod(valStr);
    }

    // Temperature
    getline(ss,valStr, ',');
    temp = stod(valStr);

    // Magnetometers
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        m[i] = stod(valStr);
    }

    // Accelerometers
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        a[i] = stod(valStr);
    }

    // Gyroscopes
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        g[i] = stod(valStr);
    }

    // Gyroscope biases ?
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        b[i] = stod(valStr);
    }
}

CObservationPtr imuData::convertToCObservation()
{
    ObservationIMUWithUncertaintyPtr obs = ObservationIMUWithUncertainty::Create();

    obs->timestamp   = std::stoull(stamp);
    switch(dataType)
    {
    case IMU_ADIS:
    {
        // Values as in Table 1 of the reference paper of the dataset
        // with accuracy = 3*sigma --> sigma^2 = accuracy^2/9.
        mrpt::math::CMatrixDouble33 staticCov, dynamicCov;
        double static_roll_var = 0.01/9., static_head_var = 0.01 /*0.09/9.*/;
        staticCov << static_roll_var , 0.              , 0.             ,
                     0.              , static_roll_var , 0.             ,
                     0.              , 0.              , static_head_var;
        obs->setStaticCov(staticCov);

        double dyn_roll_var = 0.01 /*0.09/9*/, dyn_head_var = 0.25/9.;
        dynamicCov <<dyn_roll_var , 0.           , 0.          ,
                      0.          , dyn_roll_var , 0.          ,
                      0.          , 0.           , dyn_head_var;

        obs->sensorLabel = "IMU adis";
        obs->sensorPose  = CPose3D(-0.38, 0., 0.07, M_PI_2, 0, M_PI);

        break;
    }
    case IMU_XSENS:
    {
        // Values as in Table 1 of the reference paper of the dataset
        // with accuracy = 3*sigma --> sigma^2 = accuracy^2/9.
        mrpt::math::CMatrixDouble33 staticCov, dynamicCov;
        double static_roll_var = 0.25/9., static_head_var = 1./9.;
        staticCov << static_roll_var , 0.              , 0.             ,
                     0.              , static_roll_var , 0.             ,
                     0.              , 0.              , static_head_var;
        obs->setStaticCov(staticCov);

        double dyn_var = 4./9.;
        dynamicCov << dyn_var , 0.        , 0.          ,
                      0.      , dyn_var   , 0.          ,
                      0.      , 0.        , dyn_var;

        obs->sensorLabel = "IMU xsens";
        obs->sensorPose  = CPose3D(0.1, 0., -0.16, -M_PI_2, 0, M_PI);

        break;
    }
    default:
    {
        cout << "Unknown imu model !" << endl;
        break;
    }
    }

    obs->rawMeasurements[IMU_X_ACC] = a[0];
    obs->rawMeasurements[IMU_Y_ACC] = a[1];
    obs->rawMeasurements[IMU_Z_ACC] = a[2];
    obs->rawMeasurements[IMU_YAW_VEL] = g[0];
    obs->rawMeasurements[IMU_YAW_VEL] = g[0];
    obs->rawMeasurements[IMU_PITCH_VEL] = g[1];
    obs->rawMeasurements[IMU_ROLL_VEL] = g[2];
    obs->rawMeasurements[IMU_YAW] = yaw;
    obs->rawMeasurements[IMU_PITCH] = pitch;
    obs->rawMeasurements[IMU_ROLL] = roll;
    obs->rawMeasurements[IMU_MAG_X] = m[0];
    obs->rawMeasurements[IMU_MAG_Y] = m[1];
    obs->rawMeasurements[IMU_MAG_Z] = m[2];
    obs->rawMeasurements[IMU_TEMPERATURE] = temp;
    obs->rawMeasurements[IMU_ORI_QUAT_W] = q[3];
    obs->rawMeasurements[IMU_ORI_QUAT_X] = q[0];
    obs->rawMeasurements[IMU_ORI_QUAT_Y] = q[1];
    obs->rawMeasurements[IMU_ORI_QUAT_Z] = q[2];

    return obs;
}

void imuData::dumpToConsole_()
{
    cout << "++++++++ IMU ++++++++" << endl;
    cout << "Roll/Pitch/Yaw : " << roll << " , " << pitch << " , " << yaw << endl;
    cout << "Quaternion : [ " << q[0] << " , " << q[1] << " , "<< q[2] << " , "<< q[3] << " ]" << endl;
    cout << "Temperature  : "<< temp << endl;
    cout << "Magnetometers : " << m[0] << " , " << m[1] << " , " << m[2] << endl;
    cout << "Accelerometers : " << a[0] << " , " << a[1] << " , " << a[2] << endl;
    cout << "Gyroscopes : " << g[0] << " , " << g[1] << " , " << g[2] << endl;
    cout << "Gyro biases : " <<  b[0] << " , " << b[1] << " , " << b[2] << endl;
}
