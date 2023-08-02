#include "karst_slam/sensors/MSIS_secondary.h"
#include "karst_slam/sensors/Scan.h"
#include "karst_slam/obs/ObservationMSISBeam.h"
#include <mrpt/math/ops_matrices.h>
#include <mrpt/math.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::obs;
using namespace karst_slam::sensors;

MSIS_secondary::MSIS_secondary(const string &deviceName):
    MSIS(deviceName)
{
}

MSIS_secondary::~MSIS_secondary() = default;

void MSIS_secondary::addBeam(const ObservationMSISBeamPtr &beam,
                             const CPose3DPDFGaussian &accumulatedOdoSinceLastBeam,
                             const CPose3DPDFGaussian &currentGlobalPose,
                             const CPose3DPDFGaussian &sincePrevScanRefFramePdf,
                             bool hasAccumulatedOdoSinceLastBeam)
{
    // Here assume that scan arrives in chronological order (should always be the case !)
    int beamAngleIdx = getBeamAngleIdx(beam->getAngle());

    MRPT_LOG_DEBUG_STREAM("Incoming beam angle and corresponding Id (relative): " << beam->getAngle() << " , " << beamAngleIdx );

    // First beam of a new scan ?
    if(m_currentScan.empty())
    {
        MRPT_LOG_DEBUG_STREAM("First beam of a new scan. New scan reference global pose : " << currentGlobalPose);
        m_relativePoseToPrevBeam.push_back(beamRelativePosePdf(CPose3DPDFGaussian(),true));
    }
    else
    {
        // ToDo : put the sensor poses in the config file and set it once ! (except if the sensor can move)
        // ToDo : Take in account incertitude in sensor pose ?
        // Odometry is given in the robot frame --> must express it in the sensor frame
        if(hasAccumulatedOdoSinceLastBeam)
        {
            CPose3DQuatPDFGaussian accu_quat(accumulatedOdoSinceLastBeam), accu_sensorFrame_quat = m_params.sensorPoseQuatInv + accu_quat + m_params.sensorPoseQuat;
            CPose3DPDFGaussian accumulatedOdoSinceLastBeam_sensorFrame(accu_sensorFrame_quat);
            MRPT_LOG_DEBUG_STREAM("Relative pose from the previous beam (accumulated odometry in the sensor frame) : " << accumulatedOdoSinceLastBeam_sensorFrame);
            cout << "In quaternion : " << accu_sensorFrame_quat << endl;
            m_relativePoseToPrevBeam.push_back(beamRelativePosePdf(accumulatedOdoSinceLastBeam_sensorFrame,false));
        }
        else
            m_relativePoseToPrevBeam.push_back(beamRelativePosePdf());//accumulatedOdoSinceLastBeam_sensorFrame;
    }

    // Preprocess the beam to get filtered ranges
    // Get the idx of the beam angle
    beamPoints beam_pts = fromIntensitiesToCartesianCoords(beam->getIntensities(), beamAngleIdx);
    int nPts = beam_pts.points.cols();
    MRPT_LOG_DEBUG_STREAM("After filtering, #Point : " << nPts);
    m_currentScan.insert(make_pair(m_currentScan.size(), move(beam_pts)));
    m_curScanPoints += nPts;
}

std::map<int, CPose3DPDFGaussian> MSIS_secondary::computeBeamPosesWRTReferenceFrame(CPose3DPDFGaussian &referenceFrameLocalPose)
{
    map<int, CPose3DPDFGaussian> posePerBeamWRTRefFrame;

    CPose3DPDFGaussian curPose;
    for(int i = 0; i < m_relativePoseToPrevBeam.size(); i++)
    {
        const beamRelativePosePdf& b = m_relativePoseToPrevBeam[i];
        if(!b.isNull)
            curPose += b.posePdf;

        CPose3DPDFGaussian pose(referenceFrameLocalPose);
        pose += curPose;
        posePerBeamWRTRefFrame[i] = pose;
    }

    return posePerBeamWRTRefFrame;
}

// Arguments are given by the primary MSIS
/*void MSIS_secondary::generateScan(CPose3DPDFGaussian &referenceFrameLocalPose_SensorFrame,
                                  const CPose3DPDFGaussian& referenceFrameGlobalPose_BodyFrame,
                                  const CPose3DPDFGaussian& referenceFrameToEndFrame_BodyFrame)
{
    // Take care of the effect of robot motion on the image sonar by using locally estimated odometry (generally dead-reckoning EKF with DVL+IMU)
    // As done in "Scan Matching SLAM in underwater environments", A.Mallios sec 3.3
    MRPT_LOG_DEBUG_STREAM("Generate a new full scan with " << m_curScanPoints << " points and " << m_currentScan.size() << " beams");

    // Compute PDF pose of each beam w.r.t reference frame
    map<int, CPose3DPDFGaussian> posePerBeam_inRefFrame = computeBeamPosesWRTReferenceFrame(referenceFrameLocalPose_SensorFrame);

    // Compute PDF pose of scan points w.r.t. reference frame
    for(const auto& pair : posePerBeam_inRefFrame)
        computePointPDFInRefFrame(pair.first, pair.second);

    // Set the computed scan as last scan and clear it
    // Also reset variables
    m_lastScan = ObservationMSIS_scan::Create(move(m_currentScan),
                                              m_curScanPoints,
                                              referenceFrameGlobalPose_BodyFrame,
                                              referenceFrameToEndFrame_BodyFrame,
                                              m_deviceName,
                                              m_lastScanId++);

    reset();
}*/

void MSIS_secondary::reset()
{
    MSIS::reset();
    m_relativePoseToPrevBeam.clear();
}
