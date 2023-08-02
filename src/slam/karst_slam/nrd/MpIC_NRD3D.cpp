#include "karst_slam/nrd/MpIC_NRD3D.h"
#include "karst_slam/sensors/MSIS_primary.h"
#include "karst_slam/sensors/MSIS_secondary.h"
#include "karst_slam/obs/ObservationMSISBeam.h"
#include "karst_slam/obs/ObservationOdometry.h"
#include "karst_slam/graph/PoseGraph.h"
#include "karst_slam/scanMerging/interpolationSE3.h"
#include "karst_slam/poses.h"

#include <mrpt/utils.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::nrd;
using namespace karst_slam::obs;
using namespace karst_slam::sensors;
using namespace karst_slam::scanMerging;

MpIC_NRD3D::MpIC_NRD3D(const CConfigFile& configFile,
    const string& verticalSonarLabel,
    const string& horizontalSonarLabel,
    uint64_t startTimestamp) :
    NodeRegistrationDeciderInterface("MpIC_NRD3D", configFile),
    m_verticalSonarLabel(verticalSonarLabel),
    m_horizontalSonarLabel(horizontalSonarLabel),
    m_scanFinished(false),
    m_finishedScanRegistrable(false)
{
    m_currentScanDataBuffer.startTimestamp = startTimestamp;
    m_nHorizontalSonarStepForNodeCreation = configFile.read_int("General", "horizontalStepForNodeCreation", 200, true);
    m_getOffRangeVerticalSonarLimit = configFile.read_bool("Display", "displayVerticalSonarMaxLimit", false, true);
    m_useOffRangePts = configFile.read_bool("GaussianProcess", "useOffRangePtsAsCensoredData", false, true);
}

MpIC_NRD3D::MpIC_NRD3D(const string& configFile,
    const string& verticalSonarLabel,
    const string& horizontalSonarLabel,
    uint64_t startTimestamp) :
    MpIC_NRD3D(CConfigFile(configFile), verticalSonarLabel, horizontalSonarLabel, startTimestamp)
{}

MpIC_NRD3D::~MpIC_NRD3D()
{}

void MpIC_NRD3D::setMSISPtr(const std::shared_ptr<MSIS_primary>& msis)
{
    MRPT_LOG_DEBUG_STREAM("Set MSIS ptr");
    m_msis = msis;
    m_msis->dumpParamsToConsole();
}

void MpIC_NRD3D::setMSISPtr_noLoc(const std::shared_ptr<MSIS_secondary>& msis)
{
    MRPT_LOG_DEBUG_STREAM("Set MSIS ptr (only for map)");
    m_msisMap = msis;
    m_msisMap->dumpParamsToConsole();
}

void MpIC_NRD3D::updateStateFromObservationOdometry(const ObservationOdometryPtr& obs)
{
    m_hasReceivedOdometry = true;
    m_last_odometry_increment = obs->pose_pdf;

    // Was waiting for odometry for last pose interpolation before creating the new scan ?
    if (m_scanFinished)
    {
        // For the next scan, consider the first odometry as the transformation between the finished scan last pose and 
        // this odometry. This is done while doing the interpolation in processFinishedScan() 
        m_finishedScanDataBuffer.odometries.push_back(obs);
        m_scanFinished = false;
        m_finishedScanRegistrable = true;
    }
    else
        m_currentScanDataBuffer.odometries.push_back(obs);
}

void MpIC_NRD3D::updateStateFromObservationMSISBeam(const ObservationMSISBeamPtr& obs)
{
    if (obs->sensorLabel == m_horizontalSonarLabel)
    {
        m_currentScanDataBuffer.horizontalSonarData.push_back(obs);
        m_currentScanDataBuffer.nHorizontalStep++;
    }
    else if (obs->sensorLabel == m_verticalSonarLabel)
        m_currentScanDataBuffer.verticalSonarData.push_back(obs);

    // Check if scan is finished 
    // TODO : take number of steps for full scan from config file
    // Also wait for the first measure of the next scan to also compute 
    // the interpolated odometry between prev scan last pose and this first pose of the next scan
    if (m_currentScanDataBuffer.nHorizontalStep == m_nHorizontalSonarStepForNodeCreation)
    {
        // Wait for the next odometry before registration (needed for last poses interpolation)
        m_finishedScanDataBuffer = m_currentScanDataBuffer;
        m_currentScanDataBuffer.reset();
        m_scanFinished = true;
    }
}

bool MpIC_NRD3D::checkRegistrationCondition()
{
    MRPT_START

        bool registered = false;
    // The registration condition is very simple here
    // Add a node at each full scan completion
    if (m_finishedScanRegistrable)
    {
        // Take care that the node will be created not at the current pose !!
        // But at the reference frame of the scan (center frame)

        pose_pdf_t scan_refFrameLocalPose, scan_fromRefFrameToLastOdo, scan_fromPrefNodeToCurNode;
        // Interpolations and save data to files to be processed with python script
        processFinishedScan(scan_refFrameLocalPose, scan_fromRefFrameToLastOdo, scan_fromPrefNodeToCurNode);
        registered = registerNewNodeAtEnd(scan_refFrameLocalPose,
            scan_fromRefFrameToLastOdo,
            scan_fromPrefNodeToCurNode);
        m_finishedScanRegistrable = false;
    }

    return registered;
    MRPT_END;
}

surfaceTrainingData MpIC_NRD3D::processVerticalSonarData(const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& interpolated_drPoses,
    const CPose3D& refFrame_local,
    TNodeID nodeId,
    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& offRangeVerticalSonarPoints,
    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& localPtsInRefFrame)
{
    vector<double> ranges;

    const CPose3DPDFGaussian& sensorPoseOnRobot = m_msisMap->getSensorPose();
    size_t n = m_finishedScanDataBuffer.verticalSonarData.size();
    surfaceTrainingData res;
    res.reserve(n);
    vector<CPointPDFGaussian> localCartMeasures;
    localCartMeasures.reserve(n);

    CPose3D sensorPose;
    surfaceDatum d;
    vector<CPointPDFGaussian> localCartMeas_sensorFrame;
    CPointPDFGaussian localCartMeas;
    double maxRange = m_msisMap->getParams().maxRange;
    vector<TPoint3D> offRangePts;
    offRangePts.reserve(m_finishedScanDataBuffer.verticalSonarData.size());
    for (const ObservationMSISBeamPtr& obs : m_finishedScanDataBuffer.verticalSonarData)
    {
        d.timeStamp = obs->timestamp;
        d.yaw = obs->getAngle();

        // Local cartesian measurement(s)
        vector<CPointPDFGaussian> localCartMeas_sensorFrame = m_msisMap->fromIntensitiesToCartesianCoords(obs->getIntensities(), d.yaw, ranges);

        if (!localCartMeas_sensorFrame.empty())
        {
            localCartMeas = localCartMeas_sensorFrame[0];
            localCartMeasures.push_back(localCartMeas);
            sensorPose = interpolated_drPoses.at(d.timeStamp).pose.mean + sensorPoseOnRobot.mean;
            sensorPose.composePoint(localCartMeas.mean.x(), localCartMeas.mean.y(), localCartMeas.mean.z(),
                                    d.pt.x, d.pt.y, d.pt.z);
            d.isCensored = false;
            res.addData(d);
        }
        else if (m_getOffRangeVerticalSonarLimit && !m_useOffRangePts)
        {
            mrpt::math::TPoint3D maxRange_localPt(maxRange * cos(d.yaw), maxRange * sin(d.yaw), 0.),
                maxRange_inRefFramePt;
            sensorPose = interpolated_drPoses.at(d.timeStamp).pose.mean + sensorPoseOnRobot.mean;
            sensorPose.composePoint(maxRange_localPt.x, maxRange_localPt.y, maxRange_localPt.z,
                                    maxRange_inRefFramePt.x, maxRange_inRefFramePt.y, maxRange_inRefFramePt.z);
            offRangePts.push_back(maxRange_inRefFramePt);
        }
        else if (m_useOffRangePts)
        {
            mrpt::math::TPoint3D maxRange_localPt(maxRange * cos(d.yaw), maxRange * sin(d.yaw), 0.);
            sensorPose = interpolated_drPoses.at(d.timeStamp).pose.mean + sensorPoseOnRobot.mean;
            sensorPose.composePoint(maxRange_localPt.x, maxRange_localPt.y, maxRange_localPt.z,
                d.pt.x, d.pt.y, d.pt.z);
            d.isCensored = true;
            res.addData(d);
            offRangePts.push_back(d.pt);
        }
    }

    // Points in the scan reference frame
    n = localCartMeasures.size();
    localPtsInRefFrame.resize(4, n);

    CPoint3D localPtInRefFrame;
    const vector<surfaceDatum>& verticalData = res.getData();
    for (int i = 0; i < n; i++)
    {
        const CPointPDFGaussian& localPt = localCartMeasures.at(i);
        sensorPose = interpolated_drPoses.at(verticalData[i].timeStamp).pose.mean + sensorPoseOnRobot.mean;

        auto col = localPtsInRefFrame.col(i);

        // Position in the scan first frame
        sensorPose.composePoint(localPt.mean.x(), localPt.mean.y(), localPt.mean.z(),
            col(0), col(1), col(2));

        // Position in the scan ref frame
        refFrame_local.inverseComposePoint(col(0), col(1), col(2),
            col(0), col(1), col(2));
        col(3) = 1.;
    }

    // If required for debug display, stock also vertical sonar point corresponding to max range
    if (m_getOffRangeVerticalSonarLimit)
    {
        int m = offRangePts.size();
        offRangeVerticalSonarPoints.resize(4, m);
        for (int i = 0; i < m; i++)
        {
            const TPoint3D& pt = offRangePts[i];
            auto col = offRangeVerticalSonarPoints.col(i);

            // Position in the scan ref frame
            refFrame_local.inverseComposePoint(pt.x, pt.y, pt.z,
                col(0), col(1), col(2));
            col(3) = 1.;
        }
    }

    // Save
    std::ofstream file("karstSlamData/verticalSonarData_" + to_string(nodeId) + ".txt");
    if (file.is_open())
    {
        file << "#timestamp, yaw, localPtInScan_wrtFirstFrame" << endl;
        const vector<surfaceDatum>& verticalData = res.getData();
        for (int i = 0; i < n; i++)
        {
            const surfaceDatum& d = verticalData[i];
            file << d.timeStamp << "," << d.yaw << ","
                << d.pt.x << "," << d.pt.y << "," << d.pt.z << endl;
        }
        file.close();
    }

    return res;
}

vector<horizontalSonarData> MpIC_NRD3D::processHorizontalSonarData(const map<uint64_t, trajectoryPose<CPose3DPDFGaussian>>& interpolated_drPoses,
    const CPose3D& refFrame_local,
    TNodeID nodeId,
    Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>& localPtsInRefFrame)
{
    vector<double> ranges;

    vector<horizontalSonarData> res;
    vector<CPointPDFGaussian> localCartMeasures;
    res.reserve(m_finishedScanDataBuffer.horizontalSonarData.size());

    horizontalSonarData d;
    for (const ObservationMSISBeamPtr& obs : m_finishedScanDataBuffer.horizontalSonarData)
    {
        d.timestamp = obs->timestamp;
        d.yaw = obs->getAngle();

        // Local cartesian measurement(s)
        vector<CPointPDFGaussian> localCartMeas = m_msis->fromIntensitiesToCartesianCoords(obs->getIntensities(), d.yaw, ranges);

        if (!localCartMeas.empty())
            localCartMeasures.insert(localCartMeasures.end(), localCartMeasures.begin(), localCartMeasures.end());

        d.ranges = ranges;
        res.push_back(d);
    }

    // Points in the scan reference frame
    size_t n = localCartMeasures.size();
    localPtsInRefFrame.resize(4, n);
    const CPose3DPDFGaussian& sensorPoseOnRobot = m_msisMap->getSensorPose();

    CPose3D sensorPose;
    CPoint3D localPtInRefFrame;
    for (int i = 0; i < n; i++)
    {
        const CPointPDFGaussian& localPt = localCartMeasures.at(i);
        sensorPose = interpolated_drPoses.at(res[i].timestamp).pose.mean + sensorPoseOnRobot.mean;

        auto col = localPtsInRefFrame.col(i);

        // Position in the scan first frame
        sensorPose.composePoint(localPt.mean.x(), localPt.mean.y(), localPt.mean.z(),
            col(0), col(1), col(2));

        // Position in the scan ref frame
        refFrame_local.inverseComposePoint(col(0), col(1), col(2),
            col(0), col(1), col(2));
        col(3) = 1.;
    }

    // Save
    std::ofstream file("karstSlamData/horizontalSonarData_" + to_string(nodeId) + ".txt");
    if (file.is_open())
    {
        file << "#timestamp, yaw, alpha, beta, ranges" << endl;
        for (const horizontalSonarData& d : res)
        {
            file << d.timestamp << "," << d.yaw << "," << d.alpha << "," << d.beta;
            for (double r : d.ranges)
                file << "," << r;
            file << endl;
        }
        file.close();
    }

    return res;
}

void MpIC_NRD3D::interpolateDeadReckoningPoses(TNodeID newNodeID,
    CPose3DPDFGaussian& refFrame_local,
    CPose3DPDFGaussian& refFrameToLastOdo,
    uint64_t& refFrame_timestamp)
{
    // Get all the timestamps 
    vector<uint64_t> timeStamps_v, timeStamps_h;
    size_t nVertical = m_finishedScanDataBuffer.verticalSonarData.size(),
        nHorizontal = m_finishedScanDataBuffer.horizontalSonarData.size();
    timeStamps_v.reserve(nVertical);
    timeStamps_h.reserve(nHorizontal);
    for (const ObservationMSISBeamPtr& obs : m_finishedScanDataBuffer.verticalSonarData)
        timeStamps_v.push_back(obs->timestamp);
    for (const ObservationMSISBeamPtr& obs : m_finishedScanDataBuffer.horizontalSonarData)
        timeStamps_h.push_back(obs->timestamp);

    // Dead-reckoning poses as map (timestamp, pose)
    m_lastProcessedScan->drPoses[m_finishedScanDataBuffer.startTimestamp] = trajectoryPose<CPose3DPDFGaussian>();
    CPose3DPDFGaussian drPose, poseGaussian;
    for (const ObservationOdometryPtr& obs : m_finishedScanDataBuffer.odometries)
    {
        obs->pose_pdf.getCovarianceAndMean(poseGaussian.cov, poseGaussian.mean);
        drPose += poseGaussian;
        m_lastProcessedScan->drPoses[obs->timestamp] = trajectoryPose<CPose3DPDFGaussian>(drPose);
    }

    // Interpolation
    m_lastProcessedScan->interpolated_drPoses_v = interpolateTrajectorySE3AtGivenPoses(timeStamps_v,
        m_lastProcessedScan->drPoses,
        LINEAR);
    m_lastProcessedScan->interpolated_drPoses_h = interpolateTrajectorySE3AtGivenPoses(timeStamps_h,
        m_lastProcessedScan->drPoses,
        LINEAR,
        true);

    // Get the scan reference frame (wrt to scan first pose)
    refFrame_timestamp = m_finishedScanDataBuffer.horizontalSonarData[nHorizontal / 2]->timestamp;
    refFrame_local = m_lastProcessedScan->interpolated_drPoses_h.at(refFrame_timestamp).pose;
    refFrameToLastOdo = m_lastProcessedScan->drPoses.rbegin()->second.pose - refFrame_local;
}

void MpIC_NRD3D::processFinishedScan(pose_pdf_t& scan_refFrameLocalPose,
    pose_pdf_t& scan_fromRefFrameToLastOdo,
    pose_pdf_t& scan_fromPrefNodeToCurNode)
{
    m_lastProcessedScan = make_shared<fullScanProcessedData>();
    TNodeID newNodeID = m_pose_graph->getLastNodeID();
    size_t n = m_finishedScanDataBuffer.odometries.size();

    // Get the timestamps of the two last odometry data 
    // (one just before the end of the finished scan and one just after)
    uint64_t prevTimestamp = m_finishedScanDataBuffer.odometries[n - 2]->timestamp,
        lastOdoTimestamp = m_finishedScanDataBuffer.odometries.back()->timestamp;

    // Interpolate poses at each measurements
    // Also compute the scan reference frame w.r.t to first frame 
    // (which corresponds to the pose of the first odometry data before the scan starts)
    // And the relative pose of the last odometry (after the last scan measure) w.r.t the reference frame
    CPose3DPDFGaussian scan_refFrameLocalPose_, scan_fromRefFrameToLastOdo_;
    interpolateDeadReckoningPoses(newNodeID,
        scan_refFrameLocalPose_,
        scan_fromRefFrameToLastOdo_,
        m_lastProcessedScan->refFrame_timestamp);

    // Convert to pdf with information matrix (used in the poseGraph)
    scan_refFrameLocalPose = convertToGaussianInf(scan_refFrameLocalPose_);
    scan_fromRefFrameToLastOdo = convertToGaussianInf(scan_fromRefFrameToLastOdo_);

    // Relative pose of the current scan reference frame ( corresponding to a Node in the poseGraph) 
    // w.r.t the previous scan reference frame 
    scan_fromPrefNodeToCurNode = (m_firstScan) ? scan_refFrameLocalPose : m_previousScanRefFrameToPenultimateOdo + scan_refFrameLocalPose;

    // Current global pose of the scan reference frame 
    m_lastFinishedScanPosePDF = m_pose_graph->getLastNodePosePdf_atCreation() + scan_fromPrefNodeToCurNode;

    // Compute the global pose of the current scan first frame 
    // (used for in MpIC_ERD to express sonar data in global frame for surfaceTrainingData)
    CPose3DPDFGaussian scan_refFrameLocalPose_inv = inversePosePdf(scan_refFrameLocalPose_);
    CPose3DPDFGaussian lastFinishedScanPosePDF_ = convertToGaussian(m_lastFinishedScanPosePDF);
    m_lastProcessedScan->startFrame_globalPose = lastFinishedScanPosePDF_ + scan_refFrameLocalPose_inv;
    m_lastProcessedScan->startFrame_globalPose.cov = CMatrixDouble66::Zero(); // Start frame so no covariance 

    // Generate the data from the buffered sonar measurements
    m_lastProcessedScan->verticalScanData = processVerticalSonarData(m_lastProcessedScan->interpolated_drPoses_v,
        scan_refFrameLocalPose.mean,
        newNodeID,
        m_lastProcessedScan->offRangeVerticalSonarPoints,
        m_lastProcessedScan->verticalSonarPoints);
    m_lastProcessedScan->horizontalScanData = processHorizontalSonarData(m_lastProcessedScan->interpolated_drPoses_h,
        scan_refFrameLocalPose.mean,
        newNodeID,
        m_lastProcessedScan->horizontalSonarPoints);
    m_lastProcessedScan->startPoseRelativeToPrevScanFirstFrame = m_finishedScanDataBuffer.startPoseRelativeToPrevScanFirstFrame;

    // Keep the pose of the current scan start with reference to the start of the previous scan 
    // (needed to express successive scan wrt to the first scan (for surface estimation) ) 
    m_currentScanDataBuffer.startPoseRelativeToPrevScanFirstFrame = m_lastProcessedScan->drPoses.at(prevTimestamp).pose;// + prevToLastPose;
    m_currentScanDataBuffer.startTimestamp = prevTimestamp;
    m_currentScanDataBuffer.odometries.push_back(m_finishedScanDataBuffer.odometries.back());

    // Pose from the scan reference frame (central) to the last odometry pose before the last measurement
    pose_pdf_t lastOdo_inv = inversePosePdf(m_finishedScanDataBuffer.odometries.back()->pose_pdf);
    m_previousScanRefFrameToPenultimateOdo = scan_fromRefFrameToLastOdo + lastOdo_inv;
}

bool MpIC_NRD3D::registerNewNodeAtEnd(const pose_pdf_t& scan_refFrameLocalPose,
    const pose_pdf_t& scan_fromRefFrameToLastOdo,
    const pose_pdf_t& scan_fromPrefNodeToCurNode)
{
    // Almost similar to NodeRegistrationDeciderInterface:registerNewNodeAtEnd() except that :
    //  - use the scan refFrameGlobalPose instead of m_since_prev_node_PDF
    //  - do not reset m_since_prev_node_PDF but start it from the scan refFrameGlobalPose
    MRPT_START;

    m_since_prev_node_PDF = scan_fromRefFrameToLastOdo;
    m_pose_graph->insertNode_poseRelativeToPrevious(scan_fromPrefNodeToCurNode);

    // Normally, edge are inserted as constraint measure by sensor observations (see ERD)
    // For the first edge between the root node and the first added node, edge constraint can only be the same as odometry
    TNodeID newNodeID = m_pose_graph->getLastNodeID();
    m_pose_graph->setScanDataToNode(newNodeID, m_lastProcessedScan);

    if (m_prev_registered_nodeID == m_pose_graph->getRoot())
        m_pose_graph->insertEdgeAtEnd(m_prev_registered_nodeID, newNodeID, scan_refFrameLocalPose);

    m_prev_registered_nodeID = newNodeID;
    m_firstScan = false;

    return true;
    MRPT_END;
}