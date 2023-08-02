//#include <mrpt/utils.h>
//#include <mrpt/obs/CRawlog.h>
//#include "karst_slam/obs/obs_includes.h"
//#include "karst_slam/compare.h"
#include "karst_slam/gironaDataSetLoader/gironaDataSetLoader.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::obs;

//namespace UW_caveDataSet
//{

//static const string depthSensorDataFile   = "depth_sensor.txt";
//static const string dvlDataFile           = "dvl_linkquest.txt";
//static const string imuAdisDataFile       = "imu_adis.txt";
//static const string imu_xsens_mtiDataFile = "imu_xsens_mti.txt";
//static const string odometryDataFile      = "odometry.txt";
//static const string sonarMicronDataFile   = "sonar_micron.txt";
//static const string sonarSeakingDataFile  = "sonar_seaking.txt";

//enum DATA_FROM_SENSOR{DEPTH = 0,
//                      DVL = 1,
//                      IMU_ADIS = 2,
//                      IMU_XSENS = 3,
//                      ODOMETRY = 4,
//                      SONAR_MICRON = 5,
//                      SONAR_SEAKING = 6};

//// Here both sonar have the same number of angular steps
//const int sonar_angular_nstep = 200;
//constexpr double sonar_angular_step = 2.*M_PI/(double)sonar_angular_nstep;

//// Odometry covariance not given by the dataset
//// For tests, define at hand the covariance
//// (It will not be used in the final version where the imu and dvl will be fusionned directly)
//constexpr double xy_std_inv = 3./(4e-3), xy_var_inv = xy_std_inv*xy_std_inv; // 3*sigma = 4cm
//const double z_var_inv      = 100000.;
//constexpr double angles_std_inv = 1./(2.*0.0175), angles_var_inv = angles_std_inv*angles_std_inv; // rad (in deg : 0.2)
//const Eigen::Matrix<double,6,6> odometry_inf_matrix = (Eigen::Matrix<double,6,6>() << xy_var_inv, 0.        , 0.    , 0., 0., 0.,
//                                                                                      0.        ,xy_var_inv , 0.    , 0., 0., 0.,
//                                                                                      0.        , 0.        , z_var_inv , 0., 0., 0.,
//                                                                                      0.        , 0.        , 0.    , angles_var_inv , 0. , 0.,
//                                                                                      0.        , 0.        , 0.    , 0. , angles_var_inv , 0.,
//                                                                                      0.        , 0.        , 0.    , 0. , 0. , angles_var_inv).finished();
//const Eigen::Matrix<double,6,6> odometry_cov_matrix = odometry_inf_matrix.inverse();

//struct data
//{
//    void readFromLine(stringstream& ss,
//                      DATA_FROM_SENSOR d)
//    {
//        dataType = d;

//        string valStr;
//        // Skip time and seq idx
//        getline(ss,valStr, ',');
//        getline(ss,valStr, ',');

//        // Stamp
//        getline(ss,valStr, ',');
//        stamp = valStr;

//        readFromLine_(ss);
//    }

//    virtual CObservationPtr convertToCObservation() = 0;

//    void dumpToConsole()
//    {
//        cout << "Stamp : " << stamp << endl;
//        dumpToConsole_();
//    }

//    virtual void readFromLine_(stringstream& ss) = 0;
//    virtual void dumpToConsole_() = 0;

//    string stamp;
//    DATA_FROM_SENSOR dataType;
//};

//struct depthData : data
//{
//    void readFromLine_(stringstream &ss)
//    {
//        string valStr;

//        // Depth
//        getline(ss,valStr,',');
//        depth = stod(valStr);
//    }

//    CObservationPtr convertToCObservation()
//    {
//        ObservationDepthPtr obs = ObservationDepth::Create();

//        obs->timestamp   = std::stoull(stamp); // TTimestamp is uint64 and unsigned long long has at least a size of 64
//        obs->sensorLabel = "Depth sensor";
//        obs->depth       = depth;
//        obs->depth_std   = 0.0001; // Not exact but should be very small
//        // As given in Table 2 of "Underwater caves sonar data set", A.Mallios et al.,IJRR 2017
//        // Warning : In the paper it is given roll pitch yaw
//        obs->sensorPose  = CPose3D(-0.06 , 0., -0.1, 0., 0., 0.);

//        return obs;
//    }

//    void dumpToConsole_()
//    {
//        cout << "++++++++ Depth ++++++++" << endl;
//        cout << "Depth : " << depth << endl;
//    }

//    double depth;
//};

//struct dvlData : data
//{
//    void readFromLine_(stringstream &ss)
//    {
//        string valStr;

//        // Data good
//        for(int i = 0; i < 4; i++)
//        {
//            getline(ss,valStr, ',');
//            dataGood[i] = stoi(valStr);
//        }

//        // Altitude beam
//        for(int i = 0; i < 4 ; i++)
//        {
//            getline(ss,valStr, ',');
//            altitudeBeam [i] = stod(valStr);
//        }

//        // Bottom velocity beam
//        for(int i = 0; i < 4 ; i++)
//        {
//            getline(ss,valStr, ',');
//            bottomVelocityBeam [i] = stod(valStr);
//        }

//        // Water velocity beam
//        for(int i = 0; i < 4 ; i++)
//        {
//            getline(ss,valStr, ',');
//            waterVelocityBeam [i] = stod(valStr);
//        }

//        // Water velocity credit
//        for(int i = 0; i < 4 ; i++)
//        {
//            getline(ss,valStr, ',');
//            waterVelocityCredit [i] = stoi(valStr);
//        }

//        // Velocity Inst
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            velocityInst [i] = stod(valStr);
//        }

//        // Velocity Inst flag
//        getline(ss,valStr, ',');
//        velocityInstFlag = stoi(valStr);

//        // Velocity Earth
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            velocityEarth [i] = stod(valStr);
//        }

//        // Velocity Earth flag
//        getline(ss,valStr, ',');
//        velocityEarthFlag = stoi(valStr);

//        // Water Velocity Inst
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            waterVelocityInst [i] = stod(valStr);
//        }

//        // Water Velocity Inst flag
//        getline(ss,valStr, ',');
//        waterVelocityInstFlag = stoi(valStr);

//        // Water Velocity Earth
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            waterVelocityEarth[i] = stod(valStr);
//        }

//        // Water Velocity Earth flag
//        getline(ss,valStr, ',');
//        waterVelocityEarthFlag = stoi(valStr);

//        // Skip roll,pitch,heading,altitude
//        getline(ss,valStr, ',');
//        getline(ss,valStr, ',');
//        getline(ss,valStr, ',');
//        getline(ss,valStr, ',');

//        // Temperature
//        getline(ss,valStr, ',');
//        temperature = stod(valStr);

//        // Pressure
//        getline(ss,valStr, ',');
//        pressure = stod(valStr);

//        // Salinity
//        getline(ss,valStr, ',');
//        salinity = stod(valStr);
//    }

//    CObservationPtr convertToCObservation()
//    {
//        ObservationDVLPtr obs = ObservationDVL::Create();

//        // ToDo : Need to better understand the meaning of all the data to only keep
//        // usefule information in observationDVLPtr

//        obs->timestamp   = std::stoull(stamp);
//        obs->sensorLabel = "DVL";
//        obs->setSensorPose(CPose3D(0., 0., 0., 0., 0., 0.));

//        return obs;
//    }

//    void dumpToConsole_()
//    {
//        cout << "++++++++ DVL ++++++++" << endl;
//        cout << "Data good             : ";
//        for(int i = 0; i < 4; i++)
//            cout << dataGood[i] << " , ";
//        cout << endl;

//        cout << "Altitude beam         : ";
//        for(int i = 0; i < 4; i++)
//            cout << altitudeBeam[i] << " , ";
//        cout << endl;

//        cout << "Bottom Velocity beam  : ";
//        for(int i = 0; i < 4; i++)
//            cout << bottomVelocityBeam[i] << " , ";
//        cout << endl;

//        cout << "Water Velocity beam   : ";
//        for(int i = 0; i < 4; i++)
//            cout << waterVelocityBeam[i] << " , ";
//        cout << endl;

//        cout << "Water Velocity credit : ";
//        for(int i = 0; i < 4; i++)
//            cout << waterVelocityCredit[i] << " , ";
//        cout << endl;

//        cout << "Velocity Inst         : ";
//        for(int i = 0; i < 3; i++)
//            cout << velocityInst[i] << " , ";
//        cout << endl;

//        cout << "Velocity Inst flag      : " << velocityInstFlag << endl;

//        cout << "Velocity Earth : ";
//        for(int i = 0; i < 3; i++)
//            cout << velocityEarth[i] << " , ";
//        cout << endl;

//        cout << "Velocity Earth flag     : " << velocityEarthFlag << endl;

//        cout << "Water Velocity Inst : ";
//        for(int i = 0; i < 3; i++)
//            cout << waterVelocityInst[i] << " , ";
//        cout << endl;

//        cout << "Water Velocity Inst flag : " << waterVelocityInstFlag << endl;

//        cout << "Water Velocity Earth : ";
//        for(int i = 0; i < 3; i++)
//            cout << waterVelocityEarth[i] << " , ";
//        cout << endl;

//        cout << "Temperature : " << temperature << endl;
//        cout << "Pressure    : " << pressure    << endl;
//        cout << "Salinity    : " << salinity    << endl;

//    }

//    bool dataGood [4]; // I suppose that it is the goodness of each beam measurement (there are 4 beams)
//    double altitudeBeam [4];
//    double bottomVelocityBeam [4];
//    double waterVelocityBeam [4];
//    int waterVelocityCredit [4];
//    double velocityInst[3]; // Velocities along each axis but meaning of Inst ???? ToDo : To check
//    int velocityInstFlag; // Meaning ??? ToDo : To check
//    double velocityEarth[3]; // Velocities along each axis but meaning of Inst ???? ToDo : To check
//    int velocityEarthFlag; // Meaning ?? ToDo : To check
//    double waterVelocityInst [3];
//    int waterVelocityInstFlag;
//    double waterVelocityEarth [3];
//    int waterVelocityEarthFlag;
//    double temperature;
//    double pressure;
//    double salinity;
//};

//struct imuData : data
//{
//    void readFromLine_(stringstream &ss)
//    {
//        string valStr;
//        // Roll
//        getline(ss,valStr, ',');
//        roll  = stod(valStr);

//        // Pitch
//        getline(ss,valStr, ',');
//        pitch = stod(valStr);

//        // Yaw
//        getline(ss,valStr, ',');
//        yaw   = stod(valStr);

//        // Quaternion
//        for(int i = 0; i < 4 ; i++)
//        {
//            getline(ss,valStr, ',');
//            q[i] = stod(valStr);
//        }

//        // Temperature
//        getline(ss,valStr, ',');
//        temp = stod(valStr);

//        // Magnetometers
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            m[i] = stod(valStr);
//        }

//        // Accelerometers
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            a[i] = stod(valStr);
//        }

//        // Gyroscopes
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            g[i] = stod(valStr);
//        }

//        // Gyroscope biases ?
//        for(int i = 0; i < 3 ; i++)
//        {
//            getline(ss,valStr, ',');
//            b[i] = stod(valStr);
//        }
//    }

//    CObservationPtr convertToCObservation()
//    {
//        ObservationIMUWithUncertaintyPtr obs = ObservationIMUWithUncertainty::Create();

//        obs->timestamp   = std::stoull(stamp);
//        switch(dataType)
//        {
//        case IMU_ADIS:
//        {
//            // Values as in Table 1 of the reference paper of the dataset
//            // with accuracy = 3*sigma --> sigma^2 = accuracy^2/9.
//            mrpt::math::CMatrixDouble33 staticCov, dynamicCov;
//            double static_roll_var = 0.01/9., static_head_var = 0.01 /*0.09/9.*/;
//            staticCov << static_roll_var , 0.              , 0.             ,
//                         0.              , static_roll_var , 0.             ,
//                         0.              , 0.              , static_head_var;
//            obs->setStaticCov(staticCov);

//            double dyn_roll_var = 0.01 /*0.09/9*/, dyn_head_var = 0.25/9.;
//            dynamicCov <<dyn_roll_var , 0.           , 0.          ,
//                          0.          , dyn_roll_var , 0.          ,
//                          0.          , 0.           , dyn_head_var;

//            obs->sensorLabel = "IMU adis";
//            obs->sensorPose  = CPose3D(-0.38, 0., 0.07, M_PI_2, 0, M_PI);

//            break;
//        }
//        case IMU_XSENS:
//        {
//            // Values as in Table 1 of the reference paper of the dataset
//            // with accuracy = 3*sigma --> sigma^2 = accuracy^2/9.
//            mrpt::math::CMatrixDouble33 staticCov, dynamicCov;
//            double static_roll_var = 0.25/9., static_head_var = 1./9.;
//            staticCov << static_roll_var , 0.              , 0.             ,
//                         0.              , static_roll_var , 0.             ,
//                         0.              , 0.              , static_head_var;
//            obs->setStaticCov(staticCov);

//            double dyn_var = 4./9.;
//            dynamicCov << dyn_var , 0.        , 0.          ,
//                          0.      , dyn_var   , 0.          ,
//                          0.      , 0.        , dyn_var;

//            obs->sensorLabel = "IMU xsens";
//            obs->sensorPose  = CPose3D(0.1, 0., -0.16, -M_PI_2, 0, M_PI);

//            break;
//        }
//        default:
//        {
//            cout << "Unknown imu model !" << endl;
//            break;
//        }
//        }

//        obs->rawMeasurements[IMU_X_ACC] = a[0];
//        obs->rawMeasurements[IMU_Y_ACC] = a[1];
//        obs->rawMeasurements[IMU_Z_ACC] = a[2];
//        obs->rawMeasurements[IMU_YAW_VEL] = g[0];
//        obs->rawMeasurements[IMU_YAW_VEL] = g[0];
//        obs->rawMeasurements[IMU_PITCH_VEL] = g[1];
//        obs->rawMeasurements[IMU_ROLL_VEL] = g[2];
//        obs->rawMeasurements[IMU_YAW] = yaw;
//        obs->rawMeasurements[IMU_PITCH] = pitch;
//        obs->rawMeasurements[IMU_ROLL] = roll;
//        obs->rawMeasurements[IMU_MAG_X] = m[0];
//        obs->rawMeasurements[IMU_MAG_Y] = m[1];
//        obs->rawMeasurements[IMU_MAG_Z] = m[2];
//        obs->rawMeasurements[IMU_TEMPERATURE] = temp;
//        obs->rawMeasurements[IMU_ORI_QUAT_W] = q[3];
//        obs->rawMeasurements[IMU_ORI_QUAT_X] = q[0];
//        obs->rawMeasurements[IMU_ORI_QUAT_Y] = q[1];
//        obs->rawMeasurements[IMU_ORI_QUAT_Z] = q[2];

//        return obs;
//    }

//    void dumpToConsole_()
//    {
//        cout << "++++++++ IMU ++++++++" << endl;
//        cout << "Roll/Pitch/Yaw : " << roll << " , " << pitch << " , " << yaw << endl;
//        cout << "Quaternion : [ " << q[0] << " , " << q[1] << " , "<< q[2] << " , "<< q[3] << " ]" << endl;
//        cout << "Temperature  : "<< temp << endl;
//        cout << "Magnetometers : " << m[0] << " , " << m[1] << " , " << m[2] << endl;
//        cout << "Accelerometers : " << a[0] << " , " << a[1] << " , " << a[2] << endl;
//        cout << "Gyroscopes : " << g[0] << " , " << g[1] << " , " << g[2] << endl;
//        cout << "Gyro biases : " <<  b[0] << " , " << b[1] << " , " << b[2] << endl;
//    }

//    double roll;
//    double pitch;
//    double yaw;
//    double q[4]; // quaternion : [qx qy qz qw] in logs. Take care if used with mrpt (first qw)
//    double temp;
//    double m[3]; // magneto
//    double a[3]; // accelero
//    double g[3]; // gyro
//    double b[3]; // Seems to be the estimated gyro biases (from kalman filter ?)
//};

//// Provided odometry
//// ToDo: Seems to be given by an EKF (IMU +DVL) ie dead reckoning estimation.
//struct odometry : data
//{
//    void readFromLine_(stringstream& ss)
//    {
//        string valStr;
//        // Translation
//        for(int i = 0; i < 3; i++)
//        {
//            getline(ss,valStr, ',');
//            t[i] = stod(valStr);
//        }

//        // Rotation (represented by quaternion)
//        for(int i = 0; i < 4; i++)
//        {
//            getline(ss,valStr, ',');
//            q[i] = stod(valStr);
//        }
//    }

//    CObservationPtr convertToCObservation()
//    {
//        ObservationOdometryPtr obs = ObservationOdometry::Create();

//        obs->timestamp       = std::stoull(stamp);
//        // In MRPT, quaternion are [qw qx qy qz] while [qx qy qz qw] in the dataset logs
//        obs->pose_pdf.mean   = CPose3D(mrpt::math::CQuaternionDouble(q[3], q[0], q[1], q[2]), t[0], t[1], t[2]);
//        obs->sensorLabel     = "Odometry given by dead reckoning EKF. To be used as reference.";

//        return obs;
//    }

//    void dumpToConsole_()
//    {
//        cout << "++++++++ odometry ++++++++" << endl;
//        cout << "t : ";
//        for(int i = 0; i < 3; i++)
//            cout << t[i] << " , ";
//        cout << endl;
//        cout << "q : ";
//        for(int i = 0; i < 4; i++)
//            cout << q[i] << " , ";
//        cout << endl;
//    }

//    double t[3]; // position [x y z]
//    double q[4]; // quaternion
//};

//struct odometry_relative : data
//{
//    virtual CObservationPtr convertToCObservation()
//    {
//        ObservationOdometryPtr obs = ObservationOdometry::Create();

//        obs->timestamp   = std::stoull(stamp);
//        obs->pose_pdf.mean.inverseComposeFrom(currentPose,prevPose);
//        obs->pose_pdf.cov_inv = odometry_inf_matrix;
//        obs->sensorLabel = "Relative Odometry given by dead reckoning EKF. To be used as reference.";

//        return obs;
//    }

//    void readFromLine_(stringstream& ss){}
//    void dumpToConsole_(){}

//    CPose3D currentPose;
//    CPose3D prevPose;
//};

//struct sonar_beam : data
//{
//    void readFromLine_(stringstream& ss)
//    {
//        string valStr;
//        // Skip angle_grad
//        getline(ss,valStr, ',');

//        // angle (radian)
//        getline(ss,valStr, ',');
//        angle = stod(valStr);

//        // Correspond step index
//        step_idx = (int)round(angle/sonar_angular_step);

//        // nbins
//        getline(ss,valStr, ',');
//        nbins = stoi(valStr);

//        // Max range
//        getline(ss,valStr, ',');
//        max_range = stoi(valStr);

//        // Intensities
//        intensities.resize(nbins);
//        for(int i = 0 ; i < nbins; i++)
//        {
//            getline(ss,valStr, ',');
//            intensities[i] = stoi(valStr);
//        }
//    }

//    CObservationPtr convertToCObservation()
//    {
//        ObservationMSISBeamPtr obs = ObservationMSISBeam::Create();

//        obs->timestamp   = std::stoull(stamp);
//        obs->setAngle(angle);
//        obs->setIntensities(intensities);
//        obs->setNAngularStep(sonar_angular_nstep);
//        obs->setMaxRange(max_range);

//        // In the dataset, the sonar seaking data seems wrong as there are both data for 2*Pi and 0 angles ....
//        // Ignore the 2*pi data
//        if(fabs(obs->getAngle() - 2.*M_PI) < 1e-4)
//            return CObservationPtr();

//        switch(dataType)
//        {
//        case SONAR_MICRON:
//        {
//            obs->sensorLabel = "Sonar_Micron";
//            obs->setSensorPose(CPose3D(0.1, 0., -0.42, M_PI, 0., 0.));
//            break;
//        }
//        case SONAR_SEAKING:
//        {
//            obs->sensorLabel = "Sonar_Seaking";
//            obs->setSensorPose(CPose3D(0.55, 0., -0.15, M_PI, M_PI_2, 0.));
//            break;
//        }
//        default:
//        {
//            cout << "Unknown sonar model !" << endl;
//            break;
//        }
//        }

//        return obs;
//    }

//    void dumpToConsole_()
//    {
//        cout << "++++++++ sonar_beam ++++++++" << endl;
//        cout << "Angle : " << angle << endl;
//        cout << "Step_idx : " << step_idx << endl;
//        cout << "nbins : " << nbins << endl;
//        cout << "Max range : " << max_range << endl;
//        cout << "Intensities : ";
//        for(const int& val : intensities)
//            cout << val << " , ";
//        cout << endl;
//    }

//    double angle; // rad
//    int step_idx; // corresponding step idx of the angle
//    int nbins;
//    int max_range;
//    vector<int> intensities;
//};

//} // end namespace

//inline bool fileExists(const string& fileName) {return static_cast<bool>(ifstream(fileName));}

//void checkFiles(const string& folderPath)
//{
//    using namespace UW_caveDataSet;

//    // Raise exception if missing files
//    const vector<string> fileNames = {depthSensorDataFile,
//                                      dvlDataFile,
//                                      imuAdisDataFile,
//                                      imu_xsens_mtiDataFile,
//                                      odometryDataFile,
//                                      sonarMicronDataFile,
//                                      sonarSeakingDataFile
//                                     };

//    string fullPathName;
//    for(const string& fileName : fileNames)
//    {
//        fullPathName = folderPath + "/" + fileName;
//        if(!fileExists(fullPathName))
//        {
//            string err_msg = "The file " + fullPathName + " doesn't exist.";
//            throw logic_error(err_msg);
//            break;
//        }
//    }
//}

//template<class DATA>
//void parseDataFile(const string& fileName,
//                   UW_caveDataSet::DATA_FROM_SENSOR d,
//                   multimap<string,shared_ptr<UW_caveDataSet::data>>& allDatas)
//{
//    using namespace UW_caveDataSet;

//    // Data format: time, seq idx, stamp, depth
//    ifstream file(fileName);
//    string line;
//    if(file.is_open())
//    {
//        // Skip the first line
//        getline(file,line);
//        while(getline(file,line))
//        {
//            stringstream ss(line);
//            shared_ptr<DATA> datum = make_shared<DATA>();
//            datum->readFromLine(ss,d);

//            allDatas.insert({datum->stamp,datum});
//        }
//        file.close();
//    }
//}

//// Partial specialization for odometry in order to convert it in relative odometry
//template<>
//void parseDataFile<UW_caveDataSet::odometry_relative>(const string& fileName,
//                                      UW_caveDataSet::DATA_FROM_SENSOR d,
//                                      multimap<string,shared_ptr<UW_caveDataSet::data>>& allDatas)
//{
//    using namespace UW_caveDataSet;

//    // Data format: time, seq idx, stamp, depth
//    ifstream file(fileName);
//    string line;
//    CPose3D prevPose;
//    bool hasPrev = false;
//    odometry odo;
//    if(file.is_open())
//    {
//        // Skip the first line
//        getline(file,line);
//        while(getline(file,line))
//        {
//            stringstream ss(line);
//            shared_ptr<odometry_relative> datum = make_shared<odometry_relative>();
//            odo.readFromLine(ss,d);

//            datum->currentPose = CPose3D(mrpt::math::CQuaternionDouble(odo.q[3], odo.q[0], odo.q[1], odo.q[2]), odo.t[0], odo.t[1], odo.t[2]);
//            if(hasPrev)
//            {
//                // Only insert if pose has changed
//                if(!equalPose(datum->currentPose, prevPose))
//                {
//                    datum->stamp = odo.stamp;
//                    datum->prevPose = prevPose;
//                    allDatas.insert({odo.stamp,datum});
//                }
//            }

//            prevPose = datum->currentPose;
//            hasPrev = true;
//        }
//        file.close();
//    }
//}

//void saveAsRawlogFile(const multimap<string,shared_ptr<UW_caveDataSet::data>>& allDatas,
//                      const string& output)
//{
//    using namespace mrpt::utils;
//    using namespace mrpt::obs;
//    using namespace UW_caveDataSet;

//    CFileGZOutputStream out_f(output);
//    CObservationPtr obs;
//    for(const auto& p : allDatas)
//    {
//        //p.second->convertToCObservation()->getDescriptionAsText(cout);
//        obs = p.second->convertToCObservation();
//        if(!obs.null())
//            out_f << obs;
//    }
//}

void dataset2rawlog(const string& folderPath,
                    const string& output)
{
    using namespace UW_caveDataSet;

    // Check that the folder contains all the dataset files
    checkFiles(folderPath);

    // Parse the different files
    // Put all the data in a single map with time stamp as key
    // It will automatically sort the data in chronological order
    multimap<string, shared_ptr<UW_caveDataSet::data> > allDatas;
    //parseDataFile<depthData>(folderPath  + "/" + depthSensorDataFile,DEPTH,allDatas);
    //parseDataFile<dvlData>(folderPath + "/" + dvlDataFile,DVL,allDatas);
    //parseDataFile<imuData>(folderPath + "/" + imuAdisDataFile,IMU_ADIS,allDatas); // This IMU is used for the EKF with the DVL to generate the odometry
    //parseDataFile<imuData>(folderPath + "/" + imu_xsens_mtiDataFile,IMU_XSENS,allDatas); // Xsens is the low cost IMU (could be use to test with worse imu)
    parseDataFile<odometry_relative>(folderPath + "/" + odometryDataFile,ODOMETRY,allDatas); // Convert from absolute to relative odometries
    parseDataFile<sonar_beam>(folderPath + "/" + sonarMicronDataFile,SONAR_MICRON,allDatas);
    parseDataFile<sonar_beam>(folderPath + "/" + sonarSeakingDataFile,SONAR_SEAKING,allDatas);

    // Convert to MRPT types and save as a rawlog file
    saveAsRawlogFile(allDatas,output);
}

int main(int argc, char** argv)
{
    using namespace UW_caveDataSet;
    using namespace mrpt::utils;
    using namespace mrpt::obs;
    MRPT_START
    try{
        if(argc != 3)
        {
            cerr << "Must have two arguments indicating the dataset folder path and the output path" << endl;
            return -1;
        }

        dataset2rawlog(argv[1], argv[2]);
        return 0;
    }
    catch(const exception& e)
    {
        cerr << "ERROR : " << e.what() << endl;
        return -1;
    }

    MRPT_END
}
