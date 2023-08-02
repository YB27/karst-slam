#include "karst_slam/gironaDataSetLoader/gironaDataSetLoader.h"
#include "karst_slam/scanMerging/dataForScanMerging.h"
#include "karst_slam/scanMerging/gui/scanMergingViewer_mrpt.h"
#include "karst_slam/sensors/Scan.h"
#include "karst_slam/sensors/MSIS_primary.h"
#include "karst_slam/sensors/MSIS_secondary.h"
#include "karst_slam/scanMerging/rangeMapping.h"
#include "karst_slam/scanMerging/interpolationSE3.h"
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/system/threads.h>
#include <pid/rpath.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::gui;
using namespace karst_slam::obs;
using namespace karst_slam::sensors;
using namespace karst_slam::scanMerging;
using namespace karst_slam::UW_caveDataSet;

// Stock data from sonar as we have to wait for the odometryfor pose interpolation
struct sonarDataBuffer
{
    sonarDataBuffer(vector<double> ranges_,
                 double angle_,
                 int idx_previousBodyPoseKnown_,
                 int localScanIdx_,
                 unsigned long long measureTimeStamp_)
    {
        ranges = ranges_;
        angle = angle_;
        idx_previousBodyPoseKnown = idx_previousBodyPoseKnown_;
        localScanIdx = localScanIdx_;
        measureTimeStamp = measureTimeStamp_;
    }
    vector<double> ranges;
    double angle;
    int idx_previousBodyPoseKnown;
    int localScanIdx;
    unsigned long long measureTimeStamp;
};

CPose3DPDFGaussian interpolatedPoseForSonarMeasurement(const vector<CPose3DPDFGaussian>& bodyPoses,
                                                       const vector<unsigned long long>& bodyPosesTimeStamp,
                                                       const sonarDataBuffer& sdb)
{
    // Compute the interpolated pose when the measurement has been done
    int idxPosePrev = sdb.idx_previousBodyPoseKnown, idxPoseNext = idxPosePrev + 1;

    // Manage special cases
    if(idxPoseNext == bodyPoses.size())
    {
         cout << "[interpolatedPoseForSonarMeasurement] Case where lacks next pose " << endl;
        // TODO : Make the buffer take up to the first next odometry of the next scan
         const CPose3DPDFGaussian& bodyPose_previous = bodyPoses[idxPosePrev];
         CPose3DPDFGaussian odo = bodyPose_previous - bodyPoses[idxPosePrev-1];
         double dt = (double)(bodyPosesTimeStamp[idxPosePrev] - bodyPosesTimeStamp[idxPosePrev-1]);
         double t  = (double)(sdb.measureTimeStamp - bodyPosesTimeStamp[idxPosePrev]) / dt;
         cout << "t : " << t << endl;

         return bodyPose_previous + linearInterpolateSE3(odo, CPose3DPDFGaussian(), t);
    }
    else if (idxPosePrev == 0 && bodyPosesTimeStamp[idxPosePrev] == 0)
    {
        cout << "[interpolatedPoseForSonarMeasurement] Case where lacks prev pose " << endl;
        const CPose3DPDFGaussian& bodyPose_next = bodyPoses[idxPoseNext];
        CPose3DPDFGaussian odo = bodyPoses[idxPoseNext+1] - bodyPose_next;
        double dt =(double)(bodyPosesTimeStamp[idxPoseNext + 1] - bodyPosesTimeStamp[idxPoseNext]);
        double t = (double)(bodyPosesTimeStamp[idxPoseNext] - sdb.measureTimeStamp) / dt;
        cout << "t : " << t << endl;

        return bodyPose_next - linearInterpolateSE3(odo, CPose3DPDFGaussian(), t);
    }
    else
    {
        const CPose3DPDFGaussian& bodyPose_previous = bodyPoses[idxPosePrev];
        const CPose3DPDFGaussian& bodyPose_next     = bodyPoses[idxPoseNext];
        unsigned long long bodyPose_prev_timeStamp = bodyPosesTimeStamp[idxPosePrev],
                           bodyPose_next_timeStamp = bodyPosesTimeStamp[idxPoseNext];


        double dt =(double)(bodyPose_next_timeStamp - bodyPose_prev_timeStamp);
        double t = (double)(sdb.measureTimeStamp - bodyPose_prev_timeStamp) / dt;


        return linearInterpolateSE3(bodyPose_next, bodyPose_previous, t);
    }
}

void computeHorizontalSonarDatum(int& ptIdx,
                                 shared_ptr<dataForScanMerging> dataMerging,
                                 const sonarDataBuffer& sdb,
                                 const Eigen::Matrix3d& sphericalCov_horizontalSonar,
                                 int nThetaSamples,
                                 double beamWidth,
                                 double stepThetaSample)
{
    vector<pointThetaSimu> thetaSimulated;
    for(const double r : sdb.ranges)
    {
        //cout << "Yaw : " << sdb.angle << endl;
        //cout << "Prev odo pose :" << sdb.idx_previousBodyPoseKnown << endl;
        const trajectoryPose<CPose3DPDFGaussian>& pwa = dataMerging->bodyPoses.at(sdb.measureTimeStamp);//dataMerging->posesWithAbscissaAtTimeStamp.at(sdb.measureTimeStamp);
        //cout << "Inter_pose at measure : " << pwa.pose << endl;

        //cout << "ptIdx : " << ptIdx << endl;
        //cout << "   -> Range : " << r << endl;

        sonarMeasure measure(r, sdb.angle);
        thetaSimulated = pointThetaSimu::generatePointsOnArc(measure,
                                                             pwa.pose.mean,
                                                             dataMerging->horizontalSonarPoseOnRobot,
                                                             nThetaSamples,
                                                             beamWidth,
                                                             stepThetaSample,
                                                             sphericalCov_horizontalSonar);
        dataMerging->horizontalSonarPoints_thetaVals.push_back(thetaSimulated);
        dataMerging->horizontalSonarMeasures.push_back(horizontalSonarMeasure(pwa.pose,
                                                                              ptIdx,
                                                                              r,
                                                                              sdb.angle,
                                                                              sdb.localScanIdx));
        CPoint3D pointGlobalPose = CPoint3D(fromSonarLocalCoordsToGlobalCoords(measure,
                                                                           pwa.pose.mean,
                                                                           dataMerging->horizontalSonarPoseOnRobot,
                                                                           sphericalCov_horizontalSonar).mean);
        dataMerging->horizontalSonarPoints.push_back(pointGlobalPose);
        ptIdx++;
    }
}

void computeVerticalSonarDatum(shared_ptr<dataForScanMerging> dataMerging,
                               const sonarDataBuffer& sdb,
                               const Eigen::Matrix3d& sphericalCov_verticalSonar)
{
    for(const double r : sdb.ranges)
    {
        const trajectoryPose<CPose3DPDFGaussian>& pwa = dataMerging->bodyPoses.at(sdb.measureTimeStamp);//dataMerging->posesWithAbscissaAtTimeStamp.at(sdb.measureTimeStamp);
        //cout << "   -> Range : " << r << endl;

        sonarMeasure measure(r, sdb.angle);
        dataMerging->verticalSonarPoints.push_back(fromSonarLocalCoordsToGlobalCoords(measure,
                                                                                           pwa.pose.mean,
                                                                                           dataMerging->verticalSonarPoseOnRobot,
                                                                                           sphericalCov_verticalSonar));
        surfaceDatum dat(pwa.curvilinearAbscissa,
                         sdb.angle,
                         r);
        dat.timeStamp = sdb.measureTimeStamp;
        dataMerging->trainingData.addData(move(dat));
    }
}

void computeHorizontalSonarData(shared_ptr<dataForScanMerging> dataMerging,
                                const vector<sonarDataBuffer>& horizontalSonarDataBuffer,
                                const Eigen::Matrix3d& sphericalCov_horizontalSonar,
                                int nThetaSamples,
                                double beamWidth,
                                double stepThetaSample)
{
    //cout << "############################ Horizontal sonar data ########################" << endl;
    int ptIdx = 0;
    for(const sonarDataBuffer& sdb : horizontalSonarDataBuffer)
    {
        computeHorizontalSonarDatum(ptIdx, dataMerging, sdb, sphericalCov_horizontalSonar,
                                    nThetaSamples, beamWidth, stepThetaSample);
    }
}

void computeVerticalSonarData(shared_ptr<dataForScanMerging> dataMerging,
                              const vector<sonarDataBuffer>& verticalSonarDataBuffer,
                              const Eigen::Matrix3d& sphericalCov_verticalSonar)
{
    for(const sonarDataBuffer& sdb : verticalSonarDataBuffer)
        computeVerticalSonarDatum(dataMerging, sdb, sphericalCov_verticalSonar);
}

map<unsigned long long, trajectoryPose<CPose3DPDFGaussian>> interpolatedPoseAtSonarMeasures(shared_ptr<dataForScanMerging> dataMerging,
                                                                          const vector<unsigned long long>& bodyPosesTimeStamp,
                                                                          const vector<sonarDataBuffer>& verticalSonarDataBuffer,
                                                                          const vector<sonarDataBuffer>& horizontalSonarDataBuffer)
{
    // TODO : Refactoring using interpolatedSE3.h

    map<unsigned long long, trajectoryPose<CPose3DPDFGaussian>> poses;
    /*trajectoryPose<CPose3DPDFGaussian> pwa;
    for(const sonarDataBuffer& sdb : verticalSonarDataBuffer)
    {
        pwa.pose = interpolatedPoseForSonarMeasurement(dataMerging->bodyPoses.getTrajectory(),
                                                       bodyPosesTimeStamp,
                                                       sdb);
        poses[sdb.measureTimeStamp] = pwa;
    }
    for(const sonarDataBuffer& sdb : horizontalSonarDataBuffer)
    {
        pwa.pose = interpolatedPoseForSonarMeasurement(dataMerging->bodyPoses.getTrajectory(),
                                                       bodyPosesTimeStamp,
                                                       sdb);
        poses[sdb.measureTimeStamp] = pwa;
    }

    // Curvilinear abscissa
    map<unsigned long long, trajectoryPose<CPose3DPDFGaussian>>::iterator poses_it_begin = poses.begin();

    CPose3DPDFGaussian firstPose = poses_it_begin->second.pose;
    //cout << "First pose : " << firstPose << endl;
    poses_it_begin->second.curvilinearAbscissa = 0.;
    poses_it_begin->second.pose = CPose3DPDFGaussian();
    poses_it_begin++;
    double prevCurvilinearAbscissa = 0.;
    CPose3DPDFGaussian prevPose;
    for(map<unsigned long long, trajectoryPose<CPose3DPDFGaussian>>::iterator it = poses_it_begin; it != poses.end(); it++)
    {
        it->second.pose -= firstPose;
        it->second.curvilinearAbscissa = prevCurvilinearAbscissa + localCurvilinearAbscissa(it->second.pose, prevPose);
        prevPose = it->second.pose;
        prevCurvilinearAbscissa = it->second.curvilinearAbscissa;
    }*/

    return poses;
}

void saveScans(const multimap<string, shared_ptr<UW_caveDataSet::data> >& allDatas,
               const string& outputFolder)
{
    // TODO : Should be done using the scanMerging library !
    /*
    // Config file for the dataset
    string testCfgPath = PID_PATH("test_config_file.ini"),
           simuCfgPath = PID_PATH("girona_config_file.ini");
    CConfigFile cfg(testCfgPath),
                cfg_scanMerging(simuCfgPath);

    rangeMapping::configure(cfg_scanMerging);

    // Parameter related to sensors
    double std_z     = cfg.read_double("Depth", "std_z", 0.04, true),
           std_pitch = DEG2RAD(cfg.read_double("IMU", "std_pitch", 0.3 ,true)),
           std_roll  = DEG2RAD(cfg.read_double("IMU", "std_roll", 0.3 ,true));

    // Parameters related to the scan merging
    int nThetaSamples = cfg_scanMerging.read_int("SonarHorizontal", "nThetaSamples", 200, true);
    double beamWidth = cfg_scanMerging.read_double("SonarHorizontal", "beamWidth", 1., true);
    double stepThetaSample = beamWidth / (double)nThetaSamples;

    // TODO : this should be in the test_config_file.ini
    double h_std_r     = cfg.read_double("MSIS", "Sonar_Micron_sphericalLocalCovariance_r", 0.1, true);
    double h_std_yaw   = DEG2RAD(cfg.read_double("MSIS", "Sonar_Micron_sphericalLocalCovariance_yaw" , DEG2RAD(0.1), true));
    double h_std_pitch = DEG2RAD(cfg.read_double("MSIS", "Sonar_Micron_sphericalLocalCovariance_pitch", DEG2RAD(1.), true));
    Eigen::Matrix3d sphericalCov_horizontalSonar = Eigen::Matrix3d::Zero();
    sphericalCov_horizontalSonar(0,0) = h_std_r*h_std_r;
    sphericalCov_horizontalSonar(1,1) = h_std_yaw*h_std_yaw;
    sphericalCov_horizontalSonar(2,2) = h_std_pitch*h_std_pitch;

    double v_std_r     = cfg.read_double("MSIS", "Sonar_Seaking_sphericalLocalCovariance_r", 0.1, true);
    double v_std_yaw   = DEG2RAD(cfg.read_double("MSIS", "Sonar_Seaking_sphericalLocalCovariance_yaw" , DEG2RAD(0.1), true));
    double v_std_pitch = DEG2RAD(cfg.read_double("MSIS", "Sonar_Seaking_sphericalLocalCovariance_pitch", DEG2RAD(1.), true));
    Eigen::Matrix3d sphericalCov_verticalSonar = Eigen::Matrix3d::Zero();
    sphericalCov_verticalSonar(0,0) = v_std_r*v_std_r;
    sphericalCov_verticalSonar(1,1) = v_std_yaw*v_std_yaw;
    sphericalCov_verticalSonar(2,2) = v_std_pitch*v_std_pitch;

    // Sonars
    MSIS_primary horizontalMSIS("Sonar_Micron");
    horizontalMSIS.loadParams(cfg);

    MSIS_secondary verticalMSIS("Sonar_Seaking");
    verticalMSIS.loadParams(cfg);

    CPose3DPDFGaussian currentPoseInScan, prevFirstPoseOnScan, firstPoseOnScan;
    double curCurvilinearAbscissa = 0.;

    // Keep the current depth, pitch and roll values
    double currentDepth, currentPitch, currentRoll;

    // Current data used for scan merging
    shared_ptr<dataForScanMerging> currentDataForScanMerging = make_shared<dataForScanMerging>(),
                                   previousDataForScanMerging = make_shared<dataForScanMerging>();
    currentDataForScanMerging->horizontalSonarPoseOnRobot = horizontalMSIS.getSensorPose().mean;
    currentDataForScanMerging->verticalSonarPoseOnRobot = verticalMSIS.getSensorPose().mean;
    //currentDataForScanMerging->bodyPoses.curvilinearAbscissa.push_back(curCurvilinearAbscissa);
    //currentDataForScanMerging->bodyPoses_curvilinearAbscissa.push_back(curCurvilinearAbscissa);

    vector<unsigned long long> bodyPosesTimeStamp = {0}, prev_bodyPosesTimeStamp; // The initial pose has an unknown timeStamp !

    int scanIdx = 0, localScanIdx = 0; // localScanIdx is 0 or 1 (first or second scan in the current pair)
    int currentScan_horizontalBeamIdx = 0;

    vector<sonarDataBuffer> verticalSonarDataBuffer, horizontalSonarDataBuffer,
                            prev_verticalSonarDataBuffer, prev_horizontalSonarDataBuffer;

    bool generateScanAtNextOdometry = false;
    for(const auto& d : allDatas)
    {
        const shared_ptr<UW_caveDataSet::data>& curData = d.second;

        // Here we do not need the dvl so we ignore it
        // We also only use IMU_ADIS as it is the one used by Girona in their EKF
        // It is more accurate than the IMU_XSENS. It could be interesting to also test by using this IMU instead.
        DATA_FROM_SENSOR dataType = curData->dataType;
        unsigned long long timeStamp = stoull(d.first);
        //cout << "Current data stamp / dataType : " << timeStamp << " , " << dataType << endl;
        switch(dataType)
        {
        case DEPTH:
        {
            currentDepth = dynamic_pointer_cast<depthData>(curData)->depth;
            //cout << " -> Depth data : " << currentDepth << endl;
            break;
        }
        case IMU_ADIS:
        {
            currentPitch = dynamic_pointer_cast<imuData>(curData)->pitch;
            currentRoll  = dynamic_pointer_cast<imuData>(curData)->roll;
            //cout << " -> IMU data pitch/roll : " << currentPitch << " , " << currentRoll << endl;
            break;
        }
        case ODOMETRY_ABSOLUTE:
        {
            // save as it
            shared_ptr<odometry> odo_absolute = dynamic_pointer_cast<odometry>(curData);
            CPose3D odoAbs(mrpt::math::CQuaternionDouble(odo_absolute->q[3], 
                                                         odo_absolute->q[0], 
                                                         odo_absolute->q[1], 
                                                         odo_absolute->q[2]), 
                                                         odo_absolute->t[0], 
                                                         odo_absolute->t[1], 
                                                         odo_absolute->t[2]);
            
            if(currentDataForScanMerging->odometryAbsolute.empty())
                currentDataForScanMerging->odometryAbsolute.push_back(odoAbs);
            else if(!karst_slam::utils::equalPose(currentDataForScanMerging->odometryAbsolute.back(), odoAbs))
                currentDataForScanMerging->odometryAbsolute.push_back(odoAbs);
            break;
        }
        case ODOMETRY:
        {
            // TODO : Should be refactored to use correctly the trajectoryPose struct

            //cout << " -> odometry data" << endl;

            shared_ptr<odometry_relative> odo = dynamic_pointer_cast<odometry_relative>(curData);
            //cout << "    -> currentPose : " << odo->currentPose << endl;
            //cout << "    -> prevPose    : " << odo->prevPose    << endl;
            CPose3DPDFGaussian incr(odo->currentPose - odo->prevPose, odometry_relative::odometry_cov_matrix);

            curCurvilinearAbscissa += localCurvilinearAbscissa(odo->currentPose, odo->prevPose);
            //cout << "    -> curCurvilinearAbscissa : " << curCurvilinearAbscissa << endl;

            if(generateScanAtNextOdometry)
            {
                //cout << " ---> Generate Scan " << endl;

                prev_bodyPosesTimeStamp.push_back(timeStamp);
                trajectoryPose<CPose3DPDFGaussian> trajPose(previousDataForScanMerging->bodyPoses.getTrajectory().back() + incr);
                //previousDataForScanMerging->bodyPoses.getTrajectory().push_back(previousDataForScanMerging->bodyPoses.traj.back() + incr);
                previousDataForScanMerging->bodyPoses.push_back(trajPose);
                //previousDataForScanMerging->bodyPoses.getTrajectory().insert(previousDataForScanMerging->bodyPoses.traj.begin(), prevFirstPoseOnScan);
                previousDataForScanMerging->bodyPoses.push_front(prevFirstPoseOnScan);

                // Compute data based on the raw sonar measurements
                // In particular, use interpolated pose (which explains why we need the buffer)
                previousDataForScanMerging->posesWithAbscissaAtTimeStamp = interpolatedPoseAtSonarMeasures(previousDataForScanMerging,
                                                                                                    prev_bodyPosesTimeStamp,
                                                                                                    prev_verticalSonarDataBuffer,
                                                                                                    prev_horizontalSonarDataBuffer);

                previousDataForScanMerging->bodyPoses.clear();
                //previousDataForScanMerging->bodyPoses_curvilinearAbscissa.clear();
                previousDataForScanMerging->bodyPoses.reserve(previousDataForScanMerging->posesWithAbscissaAtTimeStamp.size());
                //previousDataForScanMerging->bodyPoses_curvilinearAbscissa.reserve(previousDataForScanMerging->posesWithAbscissaAtTimeStamp.size());
                for(const auto& pair : previousDataForScanMerging->posesWithAbscissaAtTimeStamp)
                {
                    //cout << "timeStamp : " << pair.first << ", pose mean : " << pair.second.pose.mean << endl;
                    previousDataForScanMerging->bodyPoses.push_back(trajectoryPose<CPose3DPDFGaussian>(pair.second.pose),
                                                                    pair.second.curvilinearAbscissa);
                    //previousDataForScanMerging->bodyPoses.traj.push_back(pair.second.pose);
                    //previousDataForScanMerging->bodyPoses.curvilinearAbscissa.push_back(pair.second.curvilinearAbscissa);
                }

                computeVerticalSonarData(previousDataForScanMerging,
                                         prev_verticalSonarDataBuffer,
                                         sphericalCov_verticalSonar);

                computeHorizontalSonarData(previousDataForScanMerging,
                                           prev_horizontalSonarDataBuffer,
                                           sphericalCov_horizontalSonar,
                                           nThetaSamples,
                                           beamWidth,
                                           stepThetaSample);

                cout << "#################################################" << endl;
                cout << "   --> Horizontal scan pair (idx " << to_string(scanIdx) << ") has been completed " << endl;
                cout << "#################################################" << endl;

                //cout << "# VerticalPts   : " << previousDataForScanMerging->verticalSonarPoints.size() << endl;
                //cout << "# HorizontalPts : " << previousDataForScanMerging->horizontalSonarMeasures.size() << endl;

                // Create the folder for the current pair of scans
                string folder = outputFolder + "/" + to_string(scanIdx);
                string command = "mkdir " + folder;
                system(command.c_str());

                previousDataForScanMerging->process();

                // Debug
                simulationViewer_mrpt viewer("scanMergingViewer", false);
                scanMergingResults res_dummy;
                viewer.init(cfg_scanMerging);
                viewer.resize(1920,1080);
                viewer.render(previousDataForScanMerging, res_dummy);
                while(!viewer.isExit())
                {
                    viewer.queryKeyboardEvents();
                    mrpt::system::sleep(10);
                }

                previousDataForScanMerging->save(folder, false );

                scanIdx++;

                generateScanAtNextOdometry = false;
            }

            currentPoseInScan += incr;

            // Fix the cov on z, pitch and roll
            // TODO: How to mix relative (x, y) and absolute (other) data to compose the covariance ?
            // currently simplify by considering diagonal cov
            Eigen::Matrix<double,6,6> cov_tp = currentPoseInScan.cov;
            currentPoseInScan.cov = Eigen::Matrix<double,6,6>::Zero();
            currentPoseInScan.cov(0,0) = cov_tp(0,0);
            currentPoseInScan.cov(1,1) = cov_tp(1,1);
            currentPoseInScan.cov(2,2) = std_z*std_z;
            currentPoseInScan.cov(3,3) = cov_tp(3,3);
            currentPoseInScan.cov(4,4) = std_pitch*std_pitch;
            currentPoseInScan.cov(5,5) = std_roll*std_roll;
            //cout << "    -> currentPoseInScan " << currentPoseInScan << endl;

            //curCurvilinearAbscissa += localCurvilinearAbscissa(odo->currentPose, odo->prevPose);
            //cout << "    -> curCurvilinearAbscissa : " << curCurvilinearAbscissa << endl;

            currentDataForScanMerging->odometry.push_back(incr);
            currentDataForScanMerging->bodyPoses.push_back(trajectoryPose<CPose3DPDFGaussian>(currentPoseInScan));
            //currentDataForScanMerging->bodyPoses.traj.push_back(currentPoseInScan);
            bodyPosesTimeStamp.push_back(timeStamp);

            break;
        }
        // Horizontal sonar
        case SONAR_MICRON:
        {
            //cout << " -> Horizontal sonar data " << endl;
            currentScan_horizontalBeamIdx++;

            shared_ptr<sonar_beam> sonarBeamData = dynamic_pointer_cast<sonar_beam>(curData);
            vector<double> ranges = horizontalMSIS.filterIntensitiesToRanges(sonarBeamData->intensities);

            // Get the idx of the dead reckoning body poses obtained before this scan measure
            int idx_previous_bodyPoseKnown = currentDataForScanMerging->bodyPoses.size();

            horizontalSonarDataBuffer.push_back(sonarDataBuffer(ranges,
                                                             sonarBeamData->angle,
                                                             idx_previous_bodyPoseKnown,
                                                             localScanIdx,
                                                             timeStamp));

            if(currentScan_horizontalBeamIdx == horizontalMSIS.getParams().nAngleStep)
                localScanIdx = 1;

            // Two full scans have been completed
            if(currentScan_horizontalBeamIdx == 2*horizontalMSIS.getParams().nAngleStep)
            {
                // Consider the first absolute odometry (pose) as the reference ie (0,0,....0)
                vector<CPose3D> odoAbs_relativeToFirstPose;
                odoAbs_relativeToFirstPose.reserve(currentDataForScanMerging->odometryAbsolute.size());
                const CPose3D& firstPose = currentDataForScanMerging->odometryAbsolute.at(0);
                for(const CPose3D& odoAbs : currentDataForScanMerging->odometryAbsolute)
                    odoAbs_relativeToFirstPose.push_back(odoAbs - firstPose);
                currentDataForScanMerging->odometryAbsolute = odoAbs_relativeToFirstPose;

                previousDataForScanMerging = make_shared<dataForScanMerging>(*currentDataForScanMerging);
                prev_verticalSonarDataBuffer = verticalSonarDataBuffer;
                prev_horizontalSonarDataBuffer = horizontalSonarDataBuffer;
                prev_bodyPosesTimeStamp = bodyPosesTimeStamp;
                generateScanAtNextOdometry = true;

                // Reset
                prevFirstPoseOnScan = firstPoseOnScan;
                firstPoseOnScan = currentPoseInScan;
                //cout << "First pose on scan: " << firstPoseOnScan << endl;

                localScanIdx = 0;
                currentDataForScanMerging = make_shared<dataForScanMerging>();
                currentDataForScanMerging->verticalSonarPoseOnRobot = verticalMSIS.getSensorPose().mean;
                currentDataForScanMerging->horizontalSonarPoseOnRobot = horizontalMSIS.getSensorPose().mean;
                curCurvilinearAbscissa = 0.;
                currentScan_horizontalBeamIdx = 0;
                verticalSonarDataBuffer.clear();
                horizontalSonarDataBuffer.clear();

                bodyPosesTimeStamp.clear();
                bodyPosesTimeStamp.push_back(timeStamp);
            }

            break;
        }
        // Vertical sonar
        case SONAR_SEAKING:
        {
            //cout << " -> Vertical sonar data " << endl;

            // Get the idx of the dead reckoning body poses obtained before this scan measure
            int idx_previous_bodyPoseKnown = currentDataForScanMerging->bodyPoses.size();

            shared_ptr<sonar_beam> sonarBeamData = dynamic_pointer_cast<sonar_beam>(curData);
            // Only take one range (highest intensity)
            vector<double> ranges = verticalMSIS.filterIntensitiesToRanges(sonarBeamData->intensities);

            verticalSonarDataBuffer.push_back(sonarDataBuffer(ranges,
                                                           sonarBeamData->angle,
                                                           idx_previous_bodyPoseKnown,
                                                           localScanIdx,
                                                              timeStamp));
            break;
        }
        default:
        {
            cout << "Encountered an unknown or ignored datatype : " << curData->dataType << endl;
            break;
        }
        }
    }*/
}

void extractScans(const string& datasetFolder,
                  const string& outputFolder)
{
    // Check that the folder contains all the dataset files
    checkFiles(datasetFolder);

    // Parse the different files
    // Put all the data in a single map with time stamp as key
    // It will automatically sort the data in chronological order
    multimap<string, shared_ptr<UW_caveDataSet::data> > allDatas;
    parseDataFile<depthData>(datasetFolder  + "/" + depthSensorDataFile,DEPTH,allDatas);
    //parseDataFile<dvlData>(datasetFolder + "/" + dvlDataFile,DVL,allDatas);
    parseDataFile<imuData>(datasetFolder + "/" + imuAdisDataFile,IMU_ADIS,allDatas);// This IMU is used for the EKF with the DVL to generate the odometry
    //parseDataFile<imuData>(datasetFolder + "/" + imu_xsens_mtiDataFile,IMU_XSENS,allDatas);// Xsens is the low cost IMU (could be use to test with worse imu)
    parseDataFile<odometry>(datasetFolder + "/" + odometryDataFile,ODOMETRY_ABSOLUTE,allDatas);
    parseDataFile<odometry_relative>(datasetFolder + "/" + odometryDataFile,ODOMETRY,allDatas); // Convert from absolute to relative odometries
    parseDataFile<sonar_beam>(datasetFolder + "/" + sonarMicronDataFile,SONAR_MICRON,allDatas);
    parseDataFile<sonar_beam>(datasetFolder + "/" + sonarSeakingDataFile,SONAR_SEAKING,allDatas);

    // Convert to MRPT types and save as a rawlog file
    saveScans(allDatas, outputFolder);
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
            cerr << "Must have two arguments indicating the dataset folder path and the output folder" << endl;
            return -1;
        }

        extractScans(argv[1], argv[2]);
        return 0;
    }
    catch(const exception& e)
    {
        cerr << "ERROR : " << e.what() << endl;
        return -1;
    }

    MRPT_END
}
