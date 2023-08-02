#ifndef DEF_DATAFORSCANMERGING_H
#define DEF_DATAFORSCANMERGING_H

#include "surfaceTrainingData.h"
#include "surfaceValidationData.h"
#include "surfaceGP.h"
#include "ellipticCylinder.h"
#include "trajectoryPose.h"
#include <vector>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <map>

namespace karst_slam{namespace scanMerging{

/** Contains data related to "raw" horizontal sonar measure  */
struct horizontalSonarMeasure
{
    horizontalSonarMeasure() = default;

    horizontalSonarMeasure(const mrpt::poses::CPose3DPDFGaussian& pose_,
                           uint64_t timeStamp_,
                           double rho_,
                           double yaw_,
                           int scanIdx_ = 0)
    {
        robotGlobalPosePdf = pose_;
        timeStamp = timeStamp_;
        rho = rho_;
        yaw = yaw_;
        scanIdx = scanIdx_;
    }

    mrpt::poses::CPose3DPDFGaussian robotGlobalPosePdf; //!< Pose at which the beam containing the measure was taken
    double rho;
    double yaw;
    int scanIdx;
    uint64_t timeStamp;
};

/** Results from the scan merging ("fusion" of vertical and horizontal sonar data) */
struct scanMergingResults
{
    surfaceGP_data surfaceGPData; //!< Data related to the learned surface by Gaussian processus
    std::map<int,std::vector<distributionTheta>> distribution; //!< map key : pt idx, vector : theta_idx, alpha, beta
};

/**
 * Handy structure containing data used for the scan merging (GP surface regression + elevation angle estimation)
 * @todo Information is redundant between the data files ... reorganize the data !
 */
struct dataForScanMerging : public mrpt::utils::COutputLogger
{
    dataForScanMerging()
    {
        this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
    }

    void append(const dataForScanMerging& data)
    {
        bodyPoses.insert(std::begin(data.bodyPoses), std::end(data.bodyPoses));

        odometry.insert(odometry.end(),
                        data.odometry.begin(),
                        data.odometry.end());

        odometry_GT.insert(odometry_GT.end(),
                           data.odometry_GT.begin(),
                           data.odometry_GT.end());

        odometryAbsolute.insert(odometryAbsolute.end(),
                                data.odometryAbsolute.begin(),
                                data.odometryAbsolute.end());

        verticalSonarPoints.insert(verticalSonarPoints.end(),
                                   data.verticalSonarPoints.begin(),
                                   data.verticalSonarPoints.end());

        horizontalSonarPoints.insert(horizontalSonarPoints.end(),
                                     data.horizontalSonarPoints.begin(),
                                     data.horizontalSonarPoints.end());

        horizontalSonarPoints_thetaVals.insert(horizontalSonarPoints_thetaVals.end(),
                                               data.horizontalSonarPoints_thetaVals.begin(),
                                               data.horizontalSonarPoints_thetaVals.end());

        horizontalSonarMeasures.insert(horizontalSonarMeasures.end(),
                                       data.horizontalSonarMeasures.begin(),
                                       data.horizontalSonarMeasures.end());
                                       
        trainingData.append(data.trainingData);        
    }
    
    void append(dataForScanMerging&& data)
    {
        bodyPoses.insert(std::make_move_iterator(data.bodyPoses.begin()), 
                        std::make_move_iterator(data.bodyPoses.end()));

        odometry.insert(odometry.end(),
                        std::make_move_iterator(data.odometry.begin()),
                        std::make_move_iterator(data.odometry.end()));

        odometry_GT.insert(odometry_GT.end(),
                           std::make_move_iterator(data.odometry_GT.begin()),
                           std::make_move_iterator(data.odometry_GT.end()));

        odometryAbsolute.insert(odometryAbsolute.end(),
                                std::make_move_iterator(data.odometryAbsolute.begin()),
                                std::make_move_iterator(data.odometryAbsolute.end()));

        verticalSonarPoints.insert(verticalSonarPoints.end(),
                                   std::make_move_iterator(data.verticalSonarPoints.begin()),
                                   std::make_move_iterator(data.verticalSonarPoints.end()));

        horizontalSonarPoints.insert(horizontalSonarPoints.end(),
                                     std::make_move_iterator(data.horizontalSonarPoints.begin()),
                                     std::make_move_iterator(data.horizontalSonarPoints.end()));

        horizontalSonarPoints_thetaVals.insert(horizontalSonarPoints_thetaVals.end(),
                                               std::make_move_iterator(data.horizontalSonarPoints_thetaVals.begin()),
                                               std::make_move_iterator(data.horizontalSonarPoints_thetaVals.end()));

        horizontalSonarMeasures.insert(horizontalSonarMeasures.end(),
                                       std::make_move_iterator(data.horizontalSonarMeasures.begin()),
                                       std::make_move_iterator(data.horizontalSonarMeasures.end()));
        
        trainingData.append(std::move(data.trainingData));  
    }

    static void saveOdometry(const std::string& name,
                             const std::vector<mrpt::poses::CPose3DPDFGaussian>& odometry) 
    {
        std::ofstream file(name);
        if(file.is_open())
        {
            file << "# x,y,z,yaw,pitch,roll,cov_11,cov_12,...,cov_66" << std::endl;
            for(const mrpt::poses::CPose3DPDFGaussian& odo : odometry)
            {
                file << odo.mean.x() << ","
                     << odo.mean.y() << ","
                     << odo.mean.z() << ","
                     << odo.mean.yaw() << ","
                     << odo.mean.pitch() << ","
                     << odo.mean.roll();
                for(int i = 0; i < 6 ; i++)
                    for(int j = 0; j < 6; j++)
                        file << "," << odo.cov(i,j);
                file << std::endl;
            }
            file.close();
        }
        else
            std::cout << "[dataForScanMerging::saveOdometry] Can't open file " << name << std::endl;
    }

    void saveOdometryAbsolute(const std::string& name,
                              const std::vector<mrpt::poses::CPose3D>& odometry) const
    {
        std::ofstream file(name);
        if(file.is_open())
        {
            file << "# x,y,z,yaw,pitch,roll" << std::endl;
            for(const mrpt::poses::CPose3D& odo : odometry)
            {
                file << odo.x() << ","
                     << odo.y() << ","
                     << odo.z() << ","
                     << odo.yaw() << ","
                     << odo.pitch() << ","
                     << odo.roll();
                file << std::endl;
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveOdometryAbsolute] Can't open file " << name);
    }

    void saveVerticalSonarPoints(const std::string& fileName)
    {
        std::ofstream file(fileName);
        if(file.is_open())
        {
            file << "# x, y, z, cov_xx, cov_xy, cov_xz, ..., cov_zz \n";
            for(const mrpt::poses::CPointPDFGaussian& pt : verticalSonarPoints)
            {
                file << pt.mean.x() << ","
                     << pt.mean.y() << ","
                     << pt.mean.z();
                for(int i = 0; i < 3 ; i++)
                {
                    for(int j = 0; j < 3; j++)
                        file << "," << pt.cov(i,j);
                }
                file << "\n";
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveVerticalSonarPoints] Can't open file " << fileName);
    }

    void saveHorizontalSonarPoints(const std::string& fileName)
    {
        std::ofstream file(fileName);
        if(file.is_open())
        {
            file << "# x, y, z \n";
            for(const mrpt::poses::CPoint3D& pt : horizontalSonarPoints)
            {
                file << pt.x() << ","
                     << pt.y() << ","
                     << pt.z() << "\n";
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveHorizontalSonarPoints] Can't open file " << fileName);
    }

    void saveHorizontalSonarMeasures(const std::string& name) const
    {
        std::cout << "saveHorizontalSonarMeasures : " << std::endl;
        std::ofstream file(name);
        if(file.is_open())
        {
            file << "# scanIdx, timeStamp,rho, yaw, robotGlobalPosePdf \n";
            for(const horizontalSonarMeasure& meas : horizontalSonarMeasures)
            {
                file << meas.scanIdx << ","
                     << meas.timeStamp << ","
                     << meas.rho << ","
                     << meas.yaw << ",";
                for(int i = 0; i < 3 ; i++)
                    file << meas.robotGlobalPosePdf.mean.m_coords[i] << ",";

                file << meas.robotGlobalPosePdf.mean.yaw() << ","
                     << meas.robotGlobalPosePdf.mean.pitch() << ","
                     << meas.robotGlobalPosePdf.mean.roll();
                for(int i = 0; i < 6 ; i++)
                {
                    for(int j = 0; j < 6; j++)
                        file << "," << meas.robotGlobalPosePdf.cov(i,j);
                }
                file << "\n";
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveHorizontalSonarMeasures] Can't open file " << name);
    }

    void saveHorizontalThetaVals(const std::string& fileName)
    {
        std::ofstream file(fileName);
        if(file.is_open())
        {
            file << "# ArcIdx, point, theta, theta_idx \n";
            int arcIdx = 0;
            for(const std::vector<pointThetaSimu>& arc : horizontalSonarPoints_thetaVals)
            {
                for(const pointThetaSimu& pt : arc)
                {
                    file << arcIdx << ','
                         << pt.point.x() << ','
                         << pt.point.y() << ','
                         << pt.point.z() << ','
                         << pt.theta << ','
                         << pt.theta_idx << "\n";
                }
                arcIdx++;
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveHorizontalThetaVals] Can't open file " << fileName);
    }

    static void saveBodyPoses(const std::string& folder,
                              const std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>>& poses)
    {
        std::string bodyPosesFile    = folder + "/bodyPoses.txt",
                    bodyPosesFile_gt = folder + "/bodyPoses_gt.txt";
        std::ofstream file_bodyPoses(bodyPosesFile), 
                      file_bodyPoses_gt(bodyPosesFile_gt);
        if(file_bodyPoses.is_open()    && 
           file_bodyPoses_gt.is_open()) 
        {
            file_bodyPoses << "# timeStamp, pose" << "\n";
            file_bodyPoses_gt << "# timeStamp, pose" << "\n";
            for (const auto& p : poses)
            {
                file_bodyPoses << p.first << ",";
                karst_slam::utils::writePoseToFile(p.second.pose   , file_bodyPoses);
                file_bodyPoses << "\n";

                file_bodyPoses_gt << p.first << ",";
                karst_slam::utils::writePoseToFile(p.second.pose_gt, file_bodyPoses_gt);
                file_bodyPoses_gt << "\n";
            }

            file_bodyPoses.close();
            file_bodyPoses_gt.close();
        }
    }

    void save(const std::string& folder, 
              bool isSimulation = true)
    {
        saveBodyPoses(folder, bodyPoses);
        priorCylinder.save(folder + "/priorCylinder.txt");

        // GT odometry (relative displacement)
        saveOdometry(folder + "/odometryGT.txt",
                     odometry_GT);
        // Odometry measured (noisy)
        saveOdometry(folder + "/odometry.txt",
                     odometry);

        saveOdometryAbsolute(folder + "/odometryAbsolute.txt",
                             odometryAbsolute);

        trainingData.save(folder + "/surfaceTrainingData.txt");
        trainingData.saveAbscissa(folder + "/curvilinearAbscissa.txt");
        horizontalData.save(folder + "/surfaceValidationData.txt");

        saveHorizontalThetaVals(folder + "/horizontalSonarThetaVals.txt");
        saveHorizontalSonarMeasures(folder + "/horizontalSonarMeasures.txt");

        if(!isSimulation)
        {
            // Also save data on vertical and horizontal measures
            saveVerticalSonarPoints(folder + "/verticalSonarPoints.txt");
            saveHorizontalSonarPoints(folder + "/horizontalSonarPoints.txt");
        }
    }

    /** Compute the prior cylinder and process horizontal sonar data 
     * to be expressed similarly to the vertical sonar data 
     */
    void process()
    {
        // Fit a prior elliptic cylinder to the vertical sonar points
        MRPT_LOG_DEBUG_STREAM("[dataForScanMerging::process] vertical sonar points : " << verticalSonarPoints.size());
        
        // TODO : use std::reference_wrapper to avoid copy ?
        std::vector<mrpt::poses::CPose3D> poses;
        poses.reserve(bodyPoses.size());
        for(auto& p : bodyPoses)
            poses.push_back(p.second.pose.mean);

        priorCylinder.fit(verticalSonarPoints, verticalSonarPoseOnRobot, poses);

        // Method using the prior cylinder axis as a reference curve
        double min_abscissa, max_abscissa;
        trainingData.computeAbscissa(priorCylinder, min_abscissa, max_abscissa);
        horizontalData = surfaceValidationData::generateFromHorizontalSonar_guide(horizontalSonarPoints_thetaVals,
                                                                                  priorCylinder,
                                                                                  isLearningMeanFunc,
                                                                                  min_abscissa,
                                                                                  max_abscissa);

        // Below is the original method using directly the curlinear abscissa on the trajectory
        // TODO : Add an option for choosing the method

        // Compute the prior values and update the training data as y' = y - prior(x)
        // Commented out as when using cylinder axis as reference curve, prior is computed in surfaceValidationData::polarCoordsProjOnGuide
        //trainingData.computeTrainingDataWithPrior(priorCylinder,
        //                                          bodyPoses,//posesWithAbscissaAtTimeStamp,//curvilinearAbscissaPoseMap,
        //                                          verticalSonarPoseOnRobot);
        // // Compute, for each horizontal sonar measurement, the corresponding s (curvilinear abscissa)
        // // and yaw (as defined for the vertical sonar)
        // std::vector<trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> trajPoses;
        // trajPoses.reserve(bodyPoses.size());
        // for(const auto& p: bodyPoses)
        //    trajPoses.push_back(p.second);
        // horizontalData = surfaceValidationData::generateFromHorizontalSonar(trajPoses,//bodyPoses.traj,
        //                                                                    horizontalSonarPoints_thetaVals,
        //                                                                    verticalSonarPoseOnRobot,
        //                                                                    priorCylinder);
    }

    /** Save local spherical coords + robotGlobalPose of horizontal non-uniform estimated measurements */
    void saveHorizontalMeasureLocalSphericalCoords(const std::string& dataFolder,
                                                   const scanMergingResults& res)
    {
        std::string fileName = dataFolder + "/horizontalSonarEstimatedData.txt";
        std::ofstream file(fileName);
        if(file.is_open())
        {
            file << "# scanIdx, rho, yaw, alpha, beta, timeStamp, arcIdx" << std::endl;

            int nArcs = horizontalSonarPoints_thetaVals.size();
            for(int arcIdx = 0; arcIdx < nArcs; arcIdx++)
            {
                const horizontalSonarMeasure& meas = horizontalSonarMeasures[arcIdx];
                if(res.distribution.find(arcIdx) != res.distribution.end())
                {
                    const std::vector<distributionTheta>& curArcDistributions = res.distribution.at(arcIdx);
                    for(const distributionTheta& dist : curArcDistributions)
                    {
                        file << meas.scanIdx << ","
                             << meas.rho << ","
                             << meas.yaw << ","
                             << dist.alpha << ","
                             << dist.beta << ","
                             << meas.timeStamp << ","
                             << arcIdx << std::endl;
                    }
                }
                // If no distribution found, add a uniform distribution (ie alpha=beta=1)
                else
                {
                     file << meas.scanIdx << ","
                          << meas.rho << ","
                          << meas.yaw << ","
                          << 1 /*alpha*/ << ","
                          << 1 /*beta*/ << "," 
                          << meas.timeStamp << ","
                          << arcIdx << std::endl;
                }          
            }
            file.close();
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveHorizontalMeasureLocalSphericalCoords] Can't open file " << fileName);
    }

    void setScanIdx(int idx)
    {
        for(horizontalSonarMeasure& meas : horizontalSonarMeasures)
            meas.scanIdx = idx;
    }

    // TODO : refactor !!!
    std::vector<mrpt::poses::CPointPDFGaussian> verticalSonarPoints; //!< Simulated vertical sonar points (global pose)
    std::vector<mrpt::poses::CPoint3D> horizontalSonarPoints;
    std::vector<std::vector<pointThetaSimu> > horizontalSonarPoints_thetaVals; //!< For each simulated horizontal measure, sampled points in the beam
    std::vector<horizontalSonarMeasure> horizontalSonarMeasures;

    mrpt::poses::CPose3D verticalSonarPoseOnRobot;
    mrpt::poses::CPose3D horizontalSonarPoseOnRobot;
    
    std::map<uint64_t, karst_slam::scanMerging::trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> bodyPoses;
    
    std::vector<mrpt::poses::CPose3DPDFGaussian> odometry; // odometry. Only for noisy trajectories
    std::vector<mrpt::poses::CPose3DPDFGaussian> odometry_GT;
    std::vector<mrpt::poses::CPose3D> odometryAbsolute; // Absolute pose given by the data set

    surfaceTrainingData trainingData;
    surfaceValidationData horizontalData;
    ellipticCylinder priorCylinder;
    bool isLearningMeanFunc;
};

struct dataForScanMerging_loopClosure : public mrpt::utils::COutputLogger
{
    dataForScanMerging_loopClosure(const std::shared_ptr<dataForScanMerging>& firstScanGrp_,
                                   const std::shared_ptr<dataForScanMerging>& loopClosureScanGrp_,
                                   const karst_slam::scanMerging::trajectory<mrpt::poses::CPose3DPDFGaussian>& bodyPoses_deadReckoning_loop_)
    {
        firstScanGrp       = firstScanGrp_;
        loopClosureScanGrp = loopClosureScanGrp_;
        bodyPoses_deadReckoning_loop = bodyPoses_deadReckoning_loop_;
    }

    void saveHorizontalMeasureLocalSphericalCoords_merged(std::ofstream& file,
                                                          const std::shared_ptr<dataForScanMerging>& scanData,
                                                          const scanMergingResults& res,
                                                          int scanIdx_merged)
    {
        int nArcs = scanData->horizontalSonarPoints_thetaVals.size();
        for (int arcIdx = 0; arcIdx < nArcs; arcIdx++)
        {
            if (res.distribution.find(arcIdx) != res.distribution.end())
            {
                const std::vector<distributionTheta>& curArcDistributions = res.distribution.at(arcIdx);
                const horizontalSonarMeasure& meas = scanData->horizontalSonarMeasures[arcIdx];
                for (const distributionTheta& dist : curArcDistributions)
                {
                    // Save only data related to the first scan of the considered scans group
                    // (Scans are grouped when the estimated surface used for 3D point estimation is based on their combined vertical sonar points)
                    if (meas.scanIdx == 0)
                    {
                        file << scanIdx_merged << ","
                            << meas.rho << ","
                            << meas.yaw << ","
                            << dist.alpha << ","
                            << dist.beta << ","
                            << meas.timeStamp << ","
                            << arcIdx << std::endl;
                    }
                }
            }
        }
    }

    void save(const std::string& dataFolder,
              const scanMergingResults& res_firstScan,
              const scanMergingResults& res_loopClosureScan)
    {
        // Merge the first scan of the firstScanGrp and loopClosureScanGrp into a dataForScanMerging object
        // to follow how the data are represented in dataForScanMerging
        
        // Merge the odometries
        int nSize = 0.5*firstScanGrp->odometry.size(); // Just take the first scan (first half)
        std::vector<mrpt::poses::CPose3DPDFGaussian> mergedOdometry, mergedOdometry_GT;
        mergedOdometry.insert(mergedOdometry.end(),
                              firstScanGrp->odometry.begin(),
                              firstScanGrp->odometry.begin() + nSize);
        mergedOdometry.insert(mergedOdometry.end(),
                              loopClosureScanGrp->odometry.begin(),
                              loopClosureScanGrp->odometry.begin() + nSize);
        mergedOdometry_GT.insert(mergedOdometry_GT.end(),
                                 firstScanGrp->odometry_GT.begin(),
                                 firstScanGrp->odometry_GT.begin() + nSize);
        mergedOdometry_GT.insert(mergedOdometry_GT.end(),
                                 loopClosureScanGrp->odometry_GT.begin(),
                                 loopClosureScanGrp->odometry_GT.begin() + nSize);
        dataForScanMerging::saveOdometry(dataFolder + "/odometry.txt", mergedOdometry);
        dataForScanMerging::saveOdometry(dataFolder + "/odometryGT.txt", mergedOdometry_GT);

        // Merge the bodyPoses
        int nSize_bodyPoses = 0.5*firstScanGrp->bodyPoses.size();
        std::map<uint64_t, trajectoryPose<mrpt::poses::CPose3DPDFGaussian>> mergedBodyPoses;        //mergedBodyPoses.append_independant(loopClosureScanGrp->bodyPoses, 0 /*startIdx*/, nSize_bodyPoses /*endIdx*/);
        mergedBodyPoses.insert(firstScanGrp->bodyPoses.begin(), std::next(firstScanGrp->bodyPoses.begin(),nSize_bodyPoses));
        mergedBodyPoses.insert(loopClosureScanGrp->bodyPoses.begin(), std::next(loopClosureScanGrp->bodyPoses.begin(),nSize_bodyPoses));

        dataForScanMerging::saveBodyPoses(dataFolder, mergedBodyPoses);

        std::string fileName = dataFolder + "/horizontalSonarEstimatedData.txt";
        std::ofstream file(fileName);
        if(file.is_open())
        {
            file << "# scanIdx, rho, yaw, alpha, beta, robotPoseIdxInScan, arcIdx" << std::endl;
            saveHorizontalMeasureLocalSphericalCoords_merged(file, firstScanGrp, res_firstScan, 0 /*scanIdx*/);
            saveHorizontalMeasureLocalSphericalCoords_merged(file, loopClosureScanGrp, res_loopClosureScan, 1 /*scanIdx*/);
        }
        else
            MRPT_LOG_ERROR_STREAM("[dataForScanMerging::saveHorizontalMeasureLocalSphericalCoords] Can't open file " << fileName);
    }

    std::shared_ptr<dataForScanMerging> firstScanGrp;
    std::shared_ptr<dataForScanMerging> loopClosureScanGrp;
    // DeadReckoning body poses between the two scans of the loop closure
    trajectory<mrpt::poses::CPose3DPDFGaussian> bodyPoses_deadReckoning_loop;
    bool isLearningMeanFunc;
};

}
}

#endif // DEF_DATAFORSCANMERGING_H
