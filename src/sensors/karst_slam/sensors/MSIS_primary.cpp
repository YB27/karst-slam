#include "karst_slam/sensors/MSIS_primary.h"
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

MSIS_primary::MSIS_primary(const string &deviceName):
    MSIS(deviceName)
{
}

MSIS_primary::~MSIS_primary() = default;

void MSIS_primary::init()
{
    m_centerIdx    = round(0.5*m_params.nAngleStep);
    m_relativePoseToPrevBeam.resize(m_params.nAngleStep);
    MSIS::init();
}

int MSIS_primary::getOffsetBeamAngleIdx(int beamAngleIdx)const
{
   int beamAngleIdx_offset = beamAngleIdx - m_beamOffsetIdx;
   if(beamAngleIdx_offset < 0)
       beamAngleIdx_offset += m_params.nAngleStep;
   return beamAngleIdx_offset;
}

void MSIS_primary::addBeam(const ObservationMSISBeamPtr &beam,
                   const CPose3DPDFGaussian &accumulatedOdoSinceLastBeam,
                   const CPose3DPDFGaussian &currentGlobalPose,
                   const CPose3DPDFGaussian &sincePrevScanRefFramePdf,
                   bool hasAccumulatedOdoSinceLastBeam)
{
    // Here assume that scan arrives in chronological order (should always be the case !)
    int beamAngleIdx = getBeamAngleIdx(beam->getAngle());
    int beamAngleIdx_offset = getOffsetBeamAngleIdx(beamAngleIdx);

    MRPT_LOG_DEBUG_STREAM("Incoming beam angle and corresponding Id (relative): " << beam->getAngle() << " , " << beamAngleIdx << " (" << beamAngleIdx_offset << ")");

    // First beam of a new scan ?
    if(m_currentScan.empty())
    {
        MRPT_LOG_DEBUG_STREAM("First beam of a new scan. New scan reference global pose : " << currentGlobalPose);
        m_beamOffsetIdx  = beamAngleIdx;
        beamAngleIdx_offset = 0;

        m_firstBeamGlobalPoseInBodyFrame = currentGlobalPose;
        m_relativePoseToPrevBeam[0]      = beamRelativePosePdf(CPose3DPDFGaussian(),true);
        m_sincePrevScanRefFramePdf       = sincePrevScanRefFramePdf;
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
            m_relativePoseToPrevBeam[beamAngleIdx_offset] = beamRelativePosePdf(accumulatedOdoSinceLastBeam_sensorFrame,false);
            if(beamAngleIdx_offset <= m_centerIdx)
                 m_sincePrevScanRefFramePdf += accumulatedOdoSinceLastBeam;
        }
        else
            m_relativePoseToPrevBeam[beamAngleIdx_offset] = beamRelativePosePdf();//accumulatedOdoSinceLastBeam_sensorFrame;
    }

    // Preprocess the beam to get filtered ranges
    // Get the idx of the beam angle
    beamPoints beam_pts = fromIntensitiesToCartesianCoords(beam->getIntensities(), beamAngleIdx);
    int nPts = beam_pts.points.cols();
    MRPT_LOG_DEBUG_STREAM("After filtering, #Point : " << nPts);
    m_currentScan.insert(make_pair(beamAngleIdx_offset, move(beam_pts)));
    m_curScanPoints += nPts;

    // Scan complete ? Verify the condition to generate the scan
    if(m_currentScan.size() == m_params.nAngleStep)
    {
        generateFullScan();
        m_hasCompletedFullScan = true;
    }
}

map<int, CPose3DPDFGaussian> MSIS_primary::computeBeamPosesWRTReferenceFrame(CPose3DPDFGaussian& referenceFrameLocalPose)
{
    map<int, CPose3DPDFGaussian> posePerBeamWRTRefFrame;

    // Ref frame is the center of the scan
    posePerBeamWRTRefFrame[m_centerIdx] = CPose3DPDFGaussian();

    CPose3DPDFGaussian poseFromIc;
    for(int i = m_centerIdx + 1; i < m_relativePoseToPrevBeam.size(); i++)
    {
        const beamRelativePosePdf& b = m_relativePoseToPrevBeam[i];
        if(!b.isNull)
            poseFromIc += b.posePdf;

        posePerBeamWRTRefFrame[i] = poseFromIc;
    }
    m_fromLocalFrameToEndFramePosePdf = poseFromIc;

    poseFromIc = CPose3DPDFGaussian();
    for(int i = m_centerIdx - 1; i >=0; i--)
    {
        const beamRelativePosePdf& b = m_relativePoseToPrevBeam[i+1];
        if(!b.isNull)
            poseFromIc += -b.posePdf;
        posePerBeamWRTRefFrame[i] = poseFromIc;
    }

    // Also compute the global pose of the (local) scan reference frame
    poseFromIc.inverse(referenceFrameLocalPose);
    return posePerBeamWRTRefFrame;
}

void MSIS_primary::generateFullScan()
{
    // Take care of the effect of robot motion on the image sonar by using locally estimated odometry (generally dead-reckoning EKF with DVL+IMU)
    // As done in "Scan Matching SLAM in underwater environments", A.Mallios sec 3.3

    MRPT_LOG_DEBUG_STREAM("Generate a new full scan with " << m_curScanPoints << " points");

    // Compute PDF pose of each beam w.r.t reference frame Ic
    CPose3DPDFGaussian referenceFrameGlobalPose_BodyFrame;
    map<int, CPose3DPDFGaussian> posePerBeam_inRefFrame = computeBeamPosesWRTReferenceFrame(m_refFrameLocalPose);

    // referenceFrame
    CPose3DPDFGaussian referenceFrameLocalPose_BodyFrame  = m_params.sensorPose + m_refFrameLocalPose + m_params.sensorPoseInv;
    CPose3DPDFGaussian referenceFrameToEndFrame_BodyFrame = m_params.sensorPose + m_fromLocalFrameToEndFramePosePdf + m_params.sensorPoseInv;
    referenceFrameGlobalPose_BodyFrame = m_firstBeamGlobalPoseInBodyFrame + referenceFrameLocalPose_BodyFrame;

    // Compute PDF pose of scan points w.r.t. reference frame Ic
    for(const auto& pair : posePerBeam_inRefFrame)
        computePointsPDFInRefFrame(pair.first, pair.second);

    // Set the computed scan as last scan and clear it
    // Also reset variables
    m_lastScan = ObservationMSIS_scan::Create(move(m_currentScan),
                                              m_curScanPoints,
                                              referenceFrameGlobalPose_BodyFrame,
                                              referenceFrameToEndFrame_BodyFrame,
                                              m_deviceName,
                                              m_lastScanId++);

    // Express data in the body frame
    // !! Here do not take in account the incertitude on sensorPose
    // ToDo : use the code in computePointCovarianceInRefFrame
    m_lastScan->composeWithPose(m_params.sensorPose.mean);
    reset();
}
