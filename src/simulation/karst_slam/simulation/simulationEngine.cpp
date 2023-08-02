#include "karst_slam/simulation/observationSimulator_sonar.h"
#include "karst_slam/simulation/simulationEngine.h"
#include "karst_slam/simulation/iglModel.h"
#include "karst_slam/scanMerging/surfaceGP.h"
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/geometry.h>

using namespace std;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace karst_slam::sensors;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;

simulationEngine::simulationEngine(const mrpt::utils::CConfigFile& cfg,
                                   const shared_ptr<observationSimulator_sonar>& verticalSonarSimulator,
                                   const shared_ptr<observationSimulator_sonar>& horizontalSonarSimulator):
    m_params(cfg),
    m_verticalSonarSimulator(verticalSonarSimulator),
    m_horizontalSonarSimulator(horizontalSonarSimulator),
    m_timeStamp(0)
{
    karst_slam::simulation::TRAJECTORY_TYPE trajectoryType = TRAJECTORY_TYPE(cfg.read_int("Simulation", "trajectoryType", karst_slam::simulation::LINEAR_CONST_SPEED_X, true));
    switch (trajectoryType)
    {
        case LINEAR_CONST_SPEED_X:
        {
            m_trajectoryGenerator = make_shared<simulationTrajectory_linearForward>(cfg);
            break;
        }
        case CUSTOM_FILE:
        {
            m_trajectoryGenerator = make_shared<simulationTrajectory_customFile>(cfg);
            break;
        }
        case LOOP_CENTERED:
        {
            m_trajectoryGenerator = make_shared<simulationTrajectory_loopCentered>(cfg);
            static_pointer_cast<simulationTrajectory_loopCentered>(m_trajectoryGenerator)->setVerticalSonarSimulator(verticalSonarSimulator);
            break;
        }
        case LOOP_CIRCLE:
        {
            m_trajectoryGenerator = make_shared<simulationTrajectory_loop_circle>(cfg);
            static_pointer_cast<simulationTrajectory_loopCentered>(m_trajectoryGenerator)->setVerticalSonarSimulator(verticalSonarSimulator);
            break;
        }
        default:
        {
            MRPT_LOG_ERROR_STREAM("[simulationEngine::simulationEngine] Unknown trajectory type");
            break;
        }
    }
    
    m_cov_small = 1e-9*Eigen::Matrix<double,6,6>::Identity();

    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

bool simulationEngine::loopClosure(const CPose3D& robotGlobalPose,
                                   double distanceThreshold)
{
    double dist = robotGlobalPose.distance3DTo(0.,0.,0.);
    cout << "loop closure dist : " << dist << endl;
    return ((m_trajectoryGenerator->getType() == LOOP_CENTERED || m_trajectoryGenerator->getType() == LOOP_CIRCLE) 
    && dist <= distanceThreshold);
}

double simulationEngine::rotateToAlignRobotInKarst_loopClosure(const Eigen::Vector3d& cylinderAxis,
                                                             const CPose3D& lastRobotPose)
{
    // Robot should move forward
    Eigen::Vector3d axisToAlignTo = cylinderAxis;
    double axisProjOnLastRobotPose_x_axis = (lastRobotPose.getRotationMatrix().t() * cylinderAxis)(0);
    if (axisProjOnLastRobotPose_x_axis < 0)
    {
        MRPT_LOG_ERROR_STREAM("[simulationEngine::generateSimulatedData_LoopClosure] Change axis sign");
        axisToAlignTo *= -1;
    }

    cout << "Cylinder axis : " << axisToAlignTo << endl;
    double yaw = atan2(axisToAlignTo(1), axisToAlignTo(0));
    return yaw; 
}

shared_ptr<dataForScanMerging> simulationEngine::generateScan(shared_ptr<dataForScanMerging>& res_scanGrp,
                                                              const shared_ptr<surfaceGP>& surfaceEstimator,
                                                              uint pairScanIdx,
                                                              int nScanToRun,
                                                              shared_ptr<dataForScanMerging> previousScan)
{
    shared_ptr<dataForScanMerging> scanData;
    MRPT_LOG_INFO_STREAM("[simulationEngine::generateScan] Simulate data for " + to_string(pairScanIdx) + "pair of scan.");
    
    // Group scans which share the surface estimation 
    // (Note that the surface is estimated with all the vertical sonar points from the scans of the group)
    res_scanGrp = make_shared<dataForScanMerging>();
    res_scanGrp->isLearningMeanFunc = m_params.isLearnMeanFunc;
    res_scanGrp->verticalSonarPoseOnRobot   = m_verticalSonarSimulator->getSensorPoseOnRobot().mean;
    res_scanGrp->horizontalSonarPoseOnRobot = m_horizontalSonarSimulator->getSensorPoseOnRobot().mean;
    
    // previousScan is nullptr only for the (nScanToRun-1) times this function is called (ie the first scan)
    // After always make the group as following (using scan indexes): 
    // (0,1,..., nScanToRun-1), (1,...,nScanToRun), (i)
    if(previousScan != nullptr)
    {
        // Change the scanIdx from 1 to 0
        previousScan->setScanIdx(0); 
        res_scanGrp->append(*previousScan);
    }
    for(int curScanIdx = 0; curScanIdx < nScanToRun; curScanIdx++)
    {
        // TODO : use a class for guiding curve
        MRPT_LOG_INFO_STREAM("[simulationEngine::generateScan] curScanIdx " + to_string(curScanIdx));
        scanData = simulateDataForOneScan(curScanIdx, res_scanGrp->priorCylinder.getCylinderAxis());
        
        if(previousScan != nullptr)
            scanData->setScanIdx(1);
        res_scanGrp->append(*scanData);
    }

    // Process and save the results of the scan
    // TODO : Could also save the results of the first scan when estimating points on the surface estimated
    // with the previous and following vertical sonar measurements (--> more points)
    res_scanGrp->process();
    MRPT_LOG_INFO_STREAM("[simulationEngine::generateScan] Scan #" + to_string(pairScanIdx) + " processed.");
    string saveFolder = m_params.saveFolder + "/expKarstDonut/" + to_string(pairScanIdx),
           command_mkdir = "mkdir " + saveFolder;
    system(command_mkdir.c_str());
    res_scanGrp->save(m_params.saveFolder + "/expKarstDonut/" + to_string(pairScanIdx)); 

    // Estimations
    surfaceEstimator->setDataFolder(saveFolder);
    scanMergingResults res = surfaceEstimator->estimateDistributions(res_scanGrp);

    res_scanGrp->saveHorizontalMeasureLocalSphericalCoords(saveFolder, res);

    m_callbackAtScanGeneration(res_scanGrp, res);

    return scanData;
}

dataForScanMerging_loopClosure simulationEngine::generateSimulatedData_LoopClosure_allScans(const shared_ptr<surfaceGP>& surfaceEstimator)
{
    // WARNING : The method here is not robust as depending on the environment, 
    // the cylinder fitting may give locally more yaw than required to align with the karst
    // --> this leads to having the vertical sonar plane "far" from the normal section 
    //  ---> Result in robot going backward !  

    MRPT_LOG_INFO_STREAM("[simulationEngine::generateSimulatedData_LoopClosure_allScans] Start generateSimulatedData");
    // DeadReckoning body poses between the two scans of the loop closure
    trajectory<CPose3DPDFGaussian> bodyPoses_deadreckoning_loop;

    uint scanIdx = 0;
    shared_ptr<dataForScanMerging> scanData = nullptr, 
                                   res_firstScanGrp = nullptr,
                                   res_loopClosureScanGrp = nullptr,
                                   res_scanGrp = nullptr;

    // The first time, run a group of 2 scans for common surface estimation
    scanData = generateScan(res_firstScanGrp, surfaceEstimator, scanIdx++);

    // Here, turn to have the robot x axis aligned with the prior cylinder axis
    // Force the cylinder axis to be forward
    double yaw = 0.;
    if(m_trajectoryGenerator->getType() == LOOP_CENTERED)
        yaw = rotateToAlignRobotInKarst_loopClosure(res_firstScanGrp->priorCylinder.getCylinderAxis(), 
                                                    res_firstScanGrp->bodyPoses.rbegin()->second.pose.mean);

    while(!loopClosure(m_curGlobalPose_deadreckoning.pose_gt.mean, 1. /*distanceThreshold*/))
    {
        scanData = generateScan(res_scanGrp, surfaceEstimator, scanIdx++, 1, scanData);
        
        if(m_trajectoryGenerator->getType() == LOOP_CENTERED)
            yaw = rotateToAlignRobotInKarst_loopClosure(res_scanGrp->priorCylinder.getCylinderAxis(), 
                                                        res_scanGrp->bodyPoses.rbegin()->second.pose.mean);
    }
   
    MRPT_LOG_INFO_STREAM("[simulationEngine::generateSimulatedData_LoopClosure_allScans] Loop Closure found !");

    // Compute the loopClosure scan
    scanData = generateScan(res_loopClosureScanGrp, surfaceEstimator, scanIdx);

    return dataForScanMerging_loopClosure(res_firstScanGrp, 
                                          res_loopClosureScanGrp, 
                                          bodyPoses_deadreckoning_loop);
}

map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> simulationEngine::generateTrajectoryDR(int curScanIdx, 
                                                                                         vector<sonarDataForBuffer>& verticalSonarTimeStamps, 
                                                                                         vector<sonarDataForBuffer>& horizontalSonarTimeStamps)
{
    double verticalBeamAngle = 0., horizontalBeamAngle = 0.;
    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> trajectoryPoses;
    trajectoryPoses[m_timeStamp] = m_curGlobalPose_deadreckoning;
    
    // Start of a scan  : reset the covariance
    if(curScanIdx == 0)
    {
        trajectoryPoses[m_timeStamp].pose.cov    = CMatrixDouble66();
        trajectoryPoses[m_timeStamp].pose_gt.cov = CMatrixDouble66();
    }
    for (int curAngleStep_vertical = 1; curAngleStep_vertical < m_params.nStepToRun; curAngleStep_vertical++)
    {
        cout << "curAngleStep_vertical : " << curAngleStep_vertical << endl;
        m_timeStamp++;
        assert(!m_trajectoryPoses.empty());
        if (m_params.nScanDataBetweenOdoData == 1 || curAngleStep_vertical % m_params.nScanDataBetweenOdoData == 0)
            trajectoryPoses[m_timeStamp] = m_trajectoryGenerator->getPose(true /*updateOdo*/, m_timeStamp /*idx*/, trajectoryPoses.rbegin()->second);

        // TimeStamp for verticalMeasurements
        verticalSonarTimeStamps.push_back(sonarDataForBuffer(m_timeStamp, m_params.anglePerStep * curAngleStep_vertical));

        // TimeStamp for horizontalMeasurements
        // One horizontal measure for k vertical one
        if (curAngleStep_vertical % 2 == 0)
        {
            horizontalSonarTimeStamps.push_back(sonarDataForBuffer(m_timeStamp, horizontalBeamAngle));
            horizontalBeamAngle += m_params.anglePerStep;
        }

        verticalBeamAngle += m_params.anglePerStep;
    }
    trajectoryPoses[++m_timeStamp] = m_trajectoryGenerator->getPose(true /*updateOdo*/, m_timeStamp /*idx*/, trajectoryPoses.rbegin()->second);


    // Keep the current (last) dead-reckoning pose
    m_curGlobalPose_deadreckoning = trajectoryPoses.rbegin()->second;

    return trajectoryPoses;
}

shared_ptr<dataForScanMerging> simulationEngine::simulateDataForOneScan(int curScanIdx, const Eigen::Vector3d& cylinderAxis)
{
    uint64_t timeStampAtScanStart = m_timeStamp;
    trajectoryPose<CPose3DPDFGaussian> prevGlobalPose_deadreckoning = m_curGlobalPose_deadreckoning;
    shared_ptr<dataForScanMerging> simData = make_shared<dataForScanMerging>();
    simData->isLearningMeanFunc = m_params.isLearnMeanFunc;
    simData->verticalSonarPoseOnRobot = m_verticalSonarSimulator->getSensorPoseOnRobot().mean;
    simData->horizontalSonarPoseOnRobot = m_horizontalSonarSimulator->getSensorPoseOnRobot().mean;

    MRPT_LOG_INFO_STREAM("[simulationEngine::simulateDataForOneScan] Start");
    int offset = curScanIdx * m_params.nStepToRun;

    // For each measurement in a scan
    double curAngleBeam_vertical = 0., curAngleBeam_horizontal = 0.;

    // Keep data (timeStamps, angle) for vertical and horizontal sonar measurements
    vector<sonarDataForBuffer> verticalSonarDataForSimu, horizontalSonarDataForSimu;
    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> drPoses = generateTrajectoryDR(curScanIdx, verticalSonarDataForSimu, horizontalSonarDataForSimu); 

    // Now interpolate the trajectory poses at the measurements timeStamps
    map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> interposes = posesPDFInterpolationAtMeasurements(drPoses,
                                                                                                       verticalSonarDataForSimu,
                                                                                                       horizontalSonarDataForSimu);
   
    cout << "After interpolation" << endl;

    for(sonarDataForBuffer& d : verticalSonarDataForSimu)
        d.posePDFAtMeasurement = interposes[d.timeStamp];
    for(sonarDataForBuffer& d : horizontalSonarDataForSimu)
        d.posePDFAtMeasurement = interposes[d.timeStamp];

    // Simulate the measurements at the interpolated poses
    simulateVerticalSonarMeasurements(verticalSonarDataForSimu,
                                      simData);

    cout << "after vertical meas" << endl;

    simulateHorizontalSonarMeasurements(horizontalSonarDataForSimu,
                                        curScanIdx,
                                        simData);

    cout << "after horizontal meas" << endl;
    simData->bodyPoses.swap(interposes);
    return simData;
}

map<uint64_t, trajectoryPose<CPose3DPDFGaussian>> simulationEngine::posesPDFInterpolationAtMeasurements(const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& drPoses,
                                                                                                        const vector<sonarDataForBuffer>& verticalSonarDataForSimu,
                                                                                                        const vector<sonarDataForBuffer>& horizontalSonarDataForSimu) const
{
    // Get the timeStamps
    vector<uint64_t> timeStampsForInterpolation;
    timeStampsForInterpolation.reserve(verticalSonarDataForSimu.size() + horizontalSonarDataForSimu.size());
    for(const sonarDataForBuffer& d : verticalSonarDataForSimu)
        timeStampsForInterpolation.push_back(d.timeStamp);
    for(const sonarDataForBuffer& d : horizontalSonarDataForSimu)
        timeStampsForInterpolation.push_back(d.timeStamp);    

    return interpolateTrajectorySE3AtGivenPoses(timeStampsForInterpolation, 
                                                drPoses,
                                                m_params.interpolationType);
}

double simulationEngine::projectionAbscissaOnGuidingCurve(const CPointPDFGaussian& point_globalFrame, 
                                                          const Eigen::Vector3d& cylinderAxis)
{
    const CPoint3D& pt = point_globalFrame.mean;
    return pt.m_coords[0]*cylinderAxis(0) + pt.m_coords[1]*cylinderAxis(1) + pt.m_coords[2]*cylinderAxis(2);
}

void simulationEngine::simulateVerticalSonarMeasurements(const vector<sonarDataForBuffer>& data,
                                                         shared_ptr<dataForScanMerging> simData)
{
    for(const sonarDataForBuffer& d : data)
    {
        simulateVerticalSonarMeasurement(d,
                                         simData);
    }
}

void simulationEngine::simulateHorizontalSonarMeasurements(const vector<sonarDataForBuffer>& data,
                                                           int curScanIdx,
                                                           shared_ptr<dataForScanMerging> simData)
{
    for(const sonarDataForBuffer& d : data)
    {
        simulateHorizontalSonarMeasurement(d,
                                           curScanIdx,
                                           simData);
    }
}

void simulationEngine::simulateVerticalSonarMeasurement(const sonarDataForBuffer& data,
                                                        shared_ptr<dataForScanMerging> simData)
{
    CPose3DPDFGaussian sonar_v_rotation(CPose3D(0, 0, 0, mrpt::math::wrapTo2Pi(data.angle), 0, 0), m_cov_small);

    vector<sonarMeasure> verticalSonarMeasures = m_verticalSonarSimulator->simulateObservation(data.posePDFAtMeasurement.pose_gt,
                                                                                               sonar_v_rotation);

    double abscissa;
    for (const sonarMeasure& measure : verticalSonarMeasures)
    {
        CPointPDFGaussian point_globalFrame = fromSonarLocalCoordsToGlobalCoords(measure,
                                                                                      data.posePDFAtMeasurement.pose.mean,
                                                                                      m_verticalSonarSimulator->getSensorPoseOnRobot().mean,
                                                                                      m_verticalSonarSimulator->getSphericalLocalCov());
        simData->verticalSonarPoints.push_back(point_globalFrame);

        surfaceDatum dat;
        dat.yaw = measure.phi;
        dat.timeStamp = data.timeStamp;
        dat.pt = TPoint3D(point_globalFrame.mean);
        simData->trainingData.addData(move(dat));
    }
}

void simulationEngine::simulateHorizontalSonarMeasurement(const sonarDataForBuffer& data,
                                                          int curScanIdx,
                                                          shared_ptr<dataForScanMerging> simData)                           
{
    vector<pointThetaSimu> thetaSimulated;
    CPose3DPDFGaussian sonar_h_rotation(CPose3D(0, 0, 0, data.angle, 0, 0), m_cov_small);
    vector<sonarMeasure> horizontalSonarMeasurements = m_horizontalSonarSimulator->simulateObservation(data.posePDFAtMeasurement.pose_gt,
        sonar_h_rotation);

    for (const sonarMeasure& measure : horizontalSonarMeasurements)
    {
        thetaSimulated = pointThetaSimu::generatePointsOnArc(measure,
                                                             data.posePDFAtMeasurement.pose.mean,
                                                             m_horizontalSonarSimulator->getSensorPoseOnRobot().mean,
                                                             m_horizontalSonarSimulator->get_nThetaSamples(),//nThetaSamples,
                                                             m_horizontalSonarSimulator->getBeamWidth(),//beamWidth,
                                                             m_horizontalSonarSimulator->getStepThetaSample(),//stepThetaSample,
                                                             m_horizontalSonarSimulator->getSphericalLocalCov());

        simData->horizontalSonarPoints_thetaVals.push_back(thetaSimulated);

        simData->horizontalSonarMeasures.push_back(horizontalSonarMeasure(data.posePDFAtMeasurement.pose,
                                                                         data.timeStamp,
                                                                         measure.range,
                                                                         sonar_h_rotation.mean.yaw(),
                                                                         curScanIdx));

        CPointPDFGaussian point_globalFrame = fromSonarLocalCoordsToGlobalCoords(measure,
                                                                                      data.posePDFAtMeasurement.pose.mean,
                                                                                      m_horizontalSonarSimulator->getSensorPoseOnRobot().mean,
                                                                                      m_horizontalSonarSimulator->getSphericalLocalCov());

        simData->horizontalSonarPoints.push_back(move(point_globalFrame.mean));
    }
}