#include "karst_slam/sensors/MSIS.h"
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

MSIS::MSIS(const string &deviceName):
    Sensor(deviceName),
    m_lastScanId(0)
{
}

MSIS::~MSIS() = default;

void MSIS::initSphericalLocalCovariance()
{
    m_params.sphericalLocalCovariance(0,0) = (double)(m_params.rangeStd*m_params.rangeStd);
    m_params.sphericalLocalCovariance(1,1) = m_params.angle_yaw_Std*m_params.angle_yaw_Std;
    m_params.sphericalLocalCovariance(2,2) = m_params.angle_pitch_Std*m_params.angle_pitch_Std;
}

/*MSISParams*/ void MSIS::loadParams(const CConfigFile &cfg)
{
    MRPT_START

    const string section = "MSIS";

    // Default values are for the Tritech Super SeaKing DFP Profiling Sonar (1.1Mhz mode)
    // as used in "Underwater caves sonar dataset", A.Mallios, I.J. of robotics research,2017
    // Note that angle/range std are set to the increment value ie rangeStd = intensityStep, angleStd = anglePerStep = 2*pi/nAngleStep
    vector<double> sensorPose, sensorPoseCovDiag;
    cfg.read_vector<vector<double>>(section,m_deviceName + "_sensorPose", vector<double>(), sensorPose, true);
    cfg.read_vector<vector<double>>(section,m_deviceName + "_sensorPoseCovDiag", vector<double>(), sensorPoseCovDiag, true);

    CPose3D sensorPose_ypr(sensorPose[0], sensorPose[1], sensorPose[2],
                           DEG2RAD(sensorPose[3]), DEG2RAD(sensorPose[4]), DEG2RAD(sensorPose[5]));
    mrpt::math::CMatrixDouble66 sensorPose_cov = Eigen::Matrix<double,6,6>::Zero();

    for(int i = 0; i < 6; i++)
        sensorPose_cov(i,i) = sensorPoseCovDiag[i] + 1e-8; // add epsilon to avoid singular matrix
    m_params.sensorPose = CPose3DPDFGaussian(sensorPose_ypr,sensorPose_cov);
    m_params.sensorPose.inverse(m_params.sensorPoseInv);

    m_params.sensorPoseQuat = CPose3DQuatPDFGaussian(m_params.sensorPose);
    m_params.sensorPoseQuat.inverse(m_params.sensorPoseQuatInv);

    m_params.horizontalBeamWidth = DEG2RAD(cfg.read_float(section,m_deviceName + "_horizontalBeamWidth",1.f ,true));
    m_params.verticalBeamWidth   = DEG2RAD(cfg.read_float(section,m_deviceName + "_verticalBeamWidth"  ,1.f ,true));
    m_params.intensityThreshold  = cfg.read_int  (section,m_deviceName + "_intensityThreshold" ,20  ,true);
    m_params.maxRange            = cfg.read_float(section,m_deviceName + "_maxRange"           ,20.f,true);
    m_params.minRange            = cfg.read_float(section,m_deviceName + "_minRange"           ,0.5f,true);
    m_params.minDistanceFilter   = cfg.read_int  (section,m_deviceName + "_minDistanceFilter"  ,3   ,true);
    m_params.nAngleStep          = cfg.read_int  (section,m_deviceName + "_nAngleStep"         ,200 ,true);
    m_params.nIntensityBins      = cfg.read_int  (section,m_deviceName + "_nIntensityBins"     ,50  ,true);
    m_params.rangeStd            = cfg.read_float(section,m_deviceName + "_rangeStd"           ,-1.f,false);
    m_params.angle_yaw_Std       = cfg.read_float(section,m_deviceName + "_angle_yaw_Std"      ,-1  ,false);
    m_params.angle_pitch_Std     = cfg.read_float(section,m_deviceName + "_angle_pitch_Std"    ,-1  ,false);

    m_params.intensityStep       = m_params.maxRange/(float)m_params.nIntensityBins;
    if(m_params.rangeStd < 0) // no value set
        m_params.rangeStd = 0.5*m_params.intensityStep;
    if(m_params.angle_yaw_Std < 0) // no value set
        m_params.angle_yaw_Std = 2.*M_PI/(float)m_params.nAngleStep; // Should also take in account horizontal beam width
    if(m_params.angle_pitch_Std < 0) // no value set
        m_params.angle_pitch_Std = m_params.verticalBeamWidth;

    m_params.minIntensityBin     = (int)ceil(m_params.minRange/m_params.intensityStep);

    // Compute values depending on parameters
    init();

    MRPT_END
}

void MSIS::setParams(const MSISParams &params)
{
    m_params = params;
    init();
}

void MSIS::initCosSinLUT()
{
    m_cosSinLUT[0] = CosSin(0);
    double curAngle = 0;
    for(int i = 1; i < m_params.nAngleStep; i++)
    {
        curAngle += m_anglePerStep;
        m_cosSinLUT[i] = CosSin(curAngle);
    }
}

void MSIS::initCartesianLocalCovLUT()
{
    for(int i = 0; i < m_params.nAngleStep; i++)
    {
        const CosSin& cs = m_cosSinLUT[i];
        CartesianCov_lut cc_lut(m_params.sphericalLocalCovariance, cs.cos, cs.sin);
        m_cartesianCovLUT.insert(make_pair(i,move(cc_lut)));
    }
}

void MSIS::init()
{
    m_curScanPoints = 0;
    m_lastScanRegistered = false;
    m_hasCompletedFullScan = false;
    m_anglePerStep = 2.*M_PI/(double)m_params.nAngleStep;

    m_currentScan.clear();

    initSphericalLocalCovariance();
    initCosSinLUT();
    initCartesianLocalCovLUT();
}

void MSIS::reset()
{
    m_currentScan.clear();
    m_curScanPoints = 0;
}

mrpt::math::CMatrixFixedNumeric<double, 3, 6> MSIS::computePosePointCompositionJacobionWRTPose(const double& lx, const double& ly, const double& lz,
                                                                                               const double &yaw, const double &pitch, const double &roll)
{
    // copy/paste from mrpt::poses::CPose3D::composePoint()
    mrpt::math::CMatrixFixedNumeric<double, 3, 6> jacobian_pose;

    // Precompute the cos/sin values
    CosSin cs_y(yaw), cs_p(pitch), cs_r(roll);
    const double& cy = cs_y.cos;
    const double& sy = cs_y.sin;
    const double& cp = cs_p.cos;
    const double& sp = cs_p.sin;
    const double& cr = cs_r.cos;
    const double& sr = cs_r.sin;
    jacobian_pose << 1, 0, 0,
            -lx*sy*cp+ly*(-sy*sp*sr-cy*cr)+lz*(-sy*sp*cr+cy*sr),   // d_x'/d_yaw
            -lx*cy*sp+ly*(cy*cp*sr       )+lz*(cy*cp*cr      ),   // d_x'/d_pitch
            ly*(cy*sp*cr+sy*sr)+lz*(-cy*sp*sr+sy*cr),   // d_x'/d_roll
            0, 1, 0,
            lx*cy*cp+ly*(cy*sp*sr-sy*cr)+lz*(cy*sp*cr+sy*sr),   // d_y'/d_yaw
            -lx*sy*sp+ly*(sy*cp*sr)      +lz*(sy*cp*cr      ),   // d_y'/d_pitch
            ly*(sy*sp*cr-cy*sr)+lz*(-sy*sp*sr-cy*cr),   // d_y'/d_roll
            0, 0, 1,
            0,  // d_z' / d_yaw
            -lx*cp-ly*sp*sr-lz*sp*cr,  // d_z' / d_pitch
            ly*cp*cr-lz*cp*sr ; // d_z' / d_roll

    return jacobian_pose;
}

cov_t MSIS::computePointCovarianceInRefFrame(const cov_t& pointCov, const double& lx, const double& ly, const double& lz,
                                             const CPose3DPDFGaussian &poseFromIc)
{
    double yaw, pitch, roll;
    poseFromIc.getPoseMean().getYawPitchRoll(yaw, pitch, roll);

    // Pose-point composition jacobian relative to the point
    const mrpt::math::CMatrixDouble33& jacobian_point = poseFromIc.getPoseMean().getRotationMatrix();

    // Pose-point composition jacobian relative to the pose  
    mrpt::math::CMatrixFixedNumeric<double,3,6> jacobian_pose = computePosePointCompositionJacobionWRTPose(lx, ly, lz,
                                                                                                            yaw, pitch, roll);

    // Cov_inRefFrame = J_point*Cov_point*J'_point + J_pose*Cov_pose*J'_pose with ' = transpose
    cov_t covInRefFrame = Eigen::Matrix<double,3,3>::Zero();
    jacobian_point.multiply_HCHt(pointCov, covInRefFrame, false); // bool = accumulate
    jacobian_pose.multiply_HCHt(poseFromIc.getCovariance(), covInRefFrame, true);

    return covInRefFrame;
}

void MSIS::computePointsPDFInRefFrame(int beamIdx, const CPose3DPDFGaussian& beamPoseWRTRefFrame)
{
    const points_mat_t& beamPoints_localCoords = m_currentScan[beamIdx].points;
    eigenAlignedVector<cov_t>& beamPointsCov_localCoords = m_currentScan[beamIdx].cov;

    // Compute corresponding covariances in the reference frame
    // Precompute cos/sin
    double yaw, pitch, roll;
    beamPoseWRTRefFrame.getPoseMean().getYawPitchRoll(yaw, pitch, roll);

    int nPoint = beamPoints_localCoords.cols();
    cov_t newCov;
    for(int i = 0; i < nPoint; i++)
    {
        newCov = computePointCovarianceInRefFrame(beamPointsCov_localCoords[i], beamPoints_localCoords(0,i), beamPoints_localCoords(1,i), beamPoints_localCoords(2,i),
                                                  beamPoseWRTRefFrame);

        beamPointsCov_localCoords[i] = move(newCov);
    }

    // Compute points position in the reference frame
    m_currentScan[beamIdx].points = beamPoseWRTRefFrame.getPoseMean().getHomogeneousMatrixVal()*beamPoints_localCoords;
}

void MSIS::setLastScanAsRegistered(mrpt::utils::TNodeID nodeId)
{
    m_lastScanRegistered = true;
    m_hasCompletedFullScan = false;
    m_lastScan->setNodeId(nodeId);
}

vector<double> MSIS::filterIntensitiesToRanges(const std::vector<int> &intensities)
{
    vector<double> ranges;
    ranges.reserve(m_params.nIntensityBins); // Max possible size
    ASSERT_(intensities.size() == m_params.nIntensityBins)

    // Ignore the first bins until the min_range
    //cout << "Min intensity bin : " << m_params.minIntensityBin << endl;
    vector<int>::const_iterator it = intensities.begin() + m_params.minIntensityBin, it_end = intensities.end();
    size_t bin_id = (size_t)m_params.minIntensityBin;
    int distSep = m_params.minDistanceFilter + 1;
    int curGroupMaxValue = -1, curGroupMaxIdx;
    for(;it != it_end; it++)
    {
        // Keep values above some threshold
        if(*it > m_params.intensityThreshold)
        {
            distSep = 0;
            // Group values (separated by a min distance)
            if(*it > curGroupMaxValue)
            {
                curGroupMaxValue = *it;
                curGroupMaxIdx   = bin_id;
            }
        }
        else
        {
            distSep++;
            if(distSep == m_params.minDistanceFilter)
            {
                ranges.push_back(binToRange(curGroupMaxIdx));
                curGroupMaxValue = -1;
            }

        }

        bin_id++;
    }

    if(curGroupMaxValue > 0)
        ranges.push_back(binToRange(curGroupMaxIdx));

    return ranges;
}

beamPoints MSIS::fromIntensitiesToCartesianCoords(const std::vector<int> &intensities, int beamAngleIdx)
{
    // Get values above some thresholds and with a mininal distance between each other
    // As described in "Scan Matching SLAM in underwater environments", A.Mallios sec 3.1
    vector<double> ranges = filterIntensitiesToRanges(intensities);

    // Convert polar coords to cartesian (homogeneous)
    // Compute cartesian covariances
    beamPoints scan_points(ranges.size());
    int i = 0;
    const CosSin& cs = m_cosSinLUT[beamAngleIdx];
    const CartesianCov_lut& cc_lut = m_cartesianCovLUT.at(beamAngleIdx);
    for(const double& r : ranges)
    {
        scan_points.points(0,i) = r*cs.cos;
        scan_points.points(1,i) = r*cs.sin;
        scan_points.points(2,i) = 0.f;
        scan_points.points(3,i) = 1;

        scan_points.cov[i] = cc_lut.getCovariance(r);

        i++;
    }
    return scan_points;
}

vector<CPointPDFGaussian> MSIS::fromIntensitiesToCartesianCoords(const std::vector<int> &intensities, 
                                                                 double beamAngle, 
                                                                 vector<double>& ranges)
{
    int beamAngleIdx = getBeamAngleIdx(beamAngle);
    //cout << "[MSIS::fromIntensitiesToCartesianCoords] beamAngle : " << beamAngle << endl;
    // Get values above some thresholds and with a minimal distance between each other
    // As described in "Scan Matching SLAM in underwater environments", A.Mallios sec 3.1
    ranges = filterIntensitiesToRanges(intensities);

    // Convert polar coords to cartesian (homogeneous)
    // Compute cartesian covariances
    vector<CPointPDFGaussian> scan_points;
    scan_points.reserve(ranges.size());
    int i = 0;
    const CosSin& cs = m_cosSinLUT[beamAngleIdx];
    const CartesianCov_lut& cc_lut = m_cartesianCovLUT.at(beamAngleIdx);
    CPointPDFGaussian pt;
    for(const double& r : ranges)
    {
        pt.mean.m_coords[0]  = r*cs.cos;
        pt.mean.m_coords[1]  = r*cs.sin;
        pt.mean.m_coords[2]  = 0.f;

        pt.cov = cc_lut.getCovariance(r);

        scan_points.push_back(pt);
    }
    return scan_points;
}


void MSIS::dumpBeamsToConsole(const beams_t &beams)
{
    for(const auto& pair : beams)
    {
        cout << "Beam " << pair.first << endl;
        const beamPoints& scan_points = pair.second;
        const points_mat_t& points = scan_points.points;
        int nPoint = points.cols();
        for(int c = 0; c < nPoint; c++)
        {
            cout << "--> " << points(0,c) << " " << points(1,c) << " " << points(2,c) << endl;
            cout << "Cov : " << endl;
            cout << scan_points.cov[c];
            cout << endl;
        }
    }
}

// ToDo : Format to be aligned
// (Check MRPT if already exist or put in utils/)
void MSIS::dumpScanToConsole(const ObservationMSIS_scan &scan)
{
    cout << "---> Global Ref Frame Pose : " << endl;
    cout << scan.m_refFrameGlobalPoseAtCreation << endl;
    cout << "---> Points with associated cov. : " << endl;
    for(int i = 0; i < scan.m_nPoints; i++)
    {
        cout << "Pose : " << scan.m_points(0,i) << " , " << scan.m_points(1,i) <<  " , " << scan.m_points(2,i) << endl;
        cout << "Cov : "  << endl;
        cout << scan.m_cov[i] << endl;
    }
}

void MSIS::dumpParamsToConsole()const
{
    cout << "--------- MSIS params ---------"                                          << endl;
    cout << "Device name                           : " << m_deviceName                 << endl;
    cout << "Horizontal Beam Width (rad)           : " << m_params.horizontalBeamWidth << endl;
    cout << "Vertical Beam Width (rad)             : " << m_params.verticalBeamWidth   << endl;
    cout << "Max range (m)                         : " << m_params.maxRange            << endl;
    cout << "Min range (m)                         : " << m_params.minRange            << endl;
    cout << "Min intensity bin                     : " << m_params.minIntensityBin     << endl;
    cout << "#Intensity bins                       : " << m_params.nIntensityBins      << endl;
    cout << "Range per intensity bin (m)           : " << m_params.intensityStep       << endl;
    cout << "Angle between successive beams (rad)  : " << m_params.nAngleStep          << endl;
    cout << "Intensity Threshold                   : " << m_params.intensityThreshold  << endl;
    cout << "Min. distance for intensity filtering : " << m_params.minDistanceFilter   << endl;
    cout << "#Angle step                           : " << m_params.nAngleStep          << endl;
    cout << "Range std (m)                         : " << m_params.rangeStd            << endl;
    cout << "Yaw std (rad)                         : " << m_params.angle_yaw_Std       << endl;
    cout << "Ptich std (rad)                       : " << m_params.angle_pitch_Std     << endl;
}
