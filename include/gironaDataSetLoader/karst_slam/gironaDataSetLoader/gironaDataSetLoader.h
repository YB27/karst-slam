#ifndef DEf_UWDATASETLOADER_H
#define DEf_UWDATASETLOADER_H

#include "depthLogData.h"
#include "dvlLogData.h"
#include "imuLogData.h"
#include "odometryLogData.h"
#include "sonarBeamLogData.h"
#include "relativeOdometryData.h"
#include "karst_slam/compare.h"
#include <memory>

namespace karst_slam{ namespace UW_caveDataSet{
// Name of the differents sensors data files
static const std::string depthSensorDataFile   = "depth_sensor.txt";
static const std::string dvlDataFile           = "dvl_linkquest.txt";
static const std::string imuAdisDataFile       = "imu_adis.txt";
static const std::string imu_xsens_mtiDataFile = "imu_xsens_mti.txt";
static const std::string odometryDataFile      = "odometry.txt";
static const std::string sonarMicronDataFile   = "sonar_micron.txt";
static const std::string sonarSeakingDataFile  = "sonar_seaking.txt";

// Here both sonar have the same number of angular steps
int sonar_beam::sonar_angular_nstep = 200; 
double sonar_beam::sonar_angular_step = 2.*M_PI/(double)sonar_beam::sonar_angular_step;

// Odometry covariance not given by the dataset
// For tests, define at hand the covariance
// (It will not be used in the final version where the imu and dvl will be fusionned directly)
constexpr double xyz_std = 8e-2/3., xy_var = xyz_std*xyz_std; // 3*sigma = 4cm
constexpr double angles_std = 0.6*0.0175, angles_var = angles_std*angles_std; // rad (in deg : 0.2)
Eigen::Matrix<double,6,6> odometry_relative::odometry_cov_matrix = (Eigen::Matrix<double,6,6>() << xy_var, 0.        , 0.    , 0., 0., 0.,
                                                                                      0.        ,xy_var, 0.    , 0., 0., 0.,
                                                                                      0.        , 0.        , xy_var , 0., 0., 0.,
                                                                                      0.        , 0.        , 0.    , angles_var , 0. , 0.,
                                                                                      0.        , 0.        , 0.    , 0. , angles_var , 0.,
                                                                                      0.        , 0.        , 0.    , 0. , 0. , angles_var).finished();
const Eigen::Matrix<double,6,6> odometry_inf_matrix = odometry_relative::odometry_cov_matrix.inverse();


inline bool fileExists(const std::string& fileName) {return static_cast<bool>(std::ifstream(fileName));}

/** Check the existence of all required sensors data file */
void checkFiles(const std::string& folderPath)
{
    // Raise exception if missing files
    const std::vector<std::string> fileNames = {depthSensorDataFile,
                                                dvlDataFile,
                                                imuAdisDataFile,
                                                imu_xsens_mtiDataFile,
                                                odometryDataFile,
                                                sonarMicronDataFile,
                                                sonarSeakingDataFile
                                               };

    std::string fullPathName;
    for(const std::string& fileName : fileNames)
    {
        fullPathName = folderPath + "/" + fileName;
        if(!fileExists(fullPathName))
        {
            std::string err_msg = "The file " + fullPathName + " doesn't exist.";
            throw std::logic_error(err_msg);
            break;
        }
    }
}

template<class DATA>
void parseDataFile(const std::string& fileName,
                   UW_caveDataSet::DATA_FROM_SENSOR d,
                   std::multimap<std::string, std::shared_ptr<data>>& allDatas)
{
    using namespace UW_caveDataSet;

    // Data format: time, seq idx, stamp, depth
    std::ifstream file(fileName);
    std::string line;
    if(file.is_open())
    {
        // Skip the first line
        std::getline(file,line);
        while(std::getline(file,line))
        {
            std::stringstream ss(line);
            std::shared_ptr<DATA> datum = std::make_shared<DATA>();
            datum->readFromLine(ss,d);

            allDatas.insert({datum->stamp,datum});
        }
        file.close();
    }
}

// Partial specialization for odometry in order to convert it in relative odometry
template<>
void parseDataFile<karst_slam::UW_caveDataSet::odometry_relative>(const std::string& fileName,
                                                                  UW_caveDataSet::DATA_FROM_SENSOR d,
                                                                  std::multimap<std::string,std::shared_ptr<data>>& allDatas)
{
    using namespace UW_caveDataSet;

    // Data format: time, seq idx, stamp, depth
    std::ifstream file(fileName);
    std::string line;
    mrpt::poses::CPose3D prevPose;
    bool hasPrev = false;
    odometry odo;
    if(file.is_open())
    {
        // Skip the first line
        std::getline(file,line);
        while(std::getline(file,line))
        {
            std::stringstream ss(line);
            std::shared_ptr<odometry_relative> datum = std::make_shared<odometry_relative>();
            odo.readFromLine(ss,d);

            datum->currentPose = mrpt::poses::CPose3D(mrpt::math::CQuaternionDouble(odo.q[3], odo.q[0], odo.q[1], odo.q[2]), odo.t[0], odo.t[1], odo.t[2]);
            if(hasPrev)
            {
                // Only insert if pose has changed
                if(!karst_slam::utils::equalPose(datum->currentPose, prevPose))
                {
                    datum->stamp = odo.stamp;
                    datum->prevPose = prevPose;
                    datum->dataType = d;
                    allDatas.insert({odo.stamp,datum});
                }
            }

            prevPose = datum->currentPose;
            hasPrev = true;
        }
        file.close();
    }
}

void saveAsRawlogFile(const std::multimap<std::string, std::shared_ptr<data>>& allDatas,
                      const std::string& output)
{
    using namespace mrpt::utils;
    using namespace mrpt::obs;
    using namespace UW_caveDataSet;

    CFileGZOutputStream out_f(output);
    CObservationPtr obs;
    for(const auto& p : allDatas)
    {
        //p.second->convertToCObservation()->getDescriptionAsText(cout);
        obs = p.second->convertToCObservation();
        if(!obs.null())
            out_f << obs;
    }
}

}}

#endif // DEf_UWDATASETLOADER_H
