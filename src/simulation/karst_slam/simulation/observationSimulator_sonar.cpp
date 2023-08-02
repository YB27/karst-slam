#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/COutputLogger.h>
#include "karst_slam/simulation/observationSimulator_sonar.h"
#include "karst_slam/sensors/Scan.h"
#include <opencv2/core/core.hpp>

using namespace std;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace karst_slam::sensors;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;

observationSimulator_sonar::observationSimulator_sonar(const shared_ptr<igl::embree::EmbreeIntersector> &embreeIntersector,
                                                       const string& name):
    observationSimulator(embreeIntersector),
    m_name(name),
    m_gen(mt19937(m_rd())),
    m_dis_theta(uniform_int_distribution<>(0,1000))
{
    m_gen.seed(1);
    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

observationSimulator_sonar::observationSimulator_sonar(const std::shared_ptr<igl::embree::EmbreeIntersector>& embreeIntersector,
                                                       const std::string& name,
                                                       const mrpt::utils::CConfigFile& cfg):
    observationSimulator_sonar(embreeIntersector, name)
{
    loadParamFromConfigFile(cfg);
}

void observationSimulator_sonar::loadParamFromConfigFile(const CConfigFile &cfg)
{
    // ToDo : use the constructor of characteristics
    double std_r     = cfg.read_double(m_name, "sphericalLocalCovariance_r", 0.1);
    double std_yaw   = DEG2RAD(cfg.read_double(m_name, "sphericalLocalCovariance_yaw" , DEG2RAD(0.1)));
    double std_pitch = DEG2RAD(cfg.read_double(m_name, "sphericalLocalCovariance_pitch", DEG2RAD(1.)));
    Eigen::Matrix3d sphericalCov = Eigen::Matrix3d::Zero();
    sphericalCov(0,0) = std_r*std_r;
    sphericalCov(1,1) = std_yaw*std_yaw;
    sphericalCov(2,2) = std_pitch*std_pitch;

    m_characteristics = sonarCharacteristics(sphericalCov,
                                             cfg.read_double(m_name, "maxRange", 20.,true),
                                             cfg.read_double(m_name, "beamWidth", 1.,true),
                                             cfg.read_bool  (m_name, "ignoreBeamWidth", true),
                                             cfg.read_int   (m_name, "nBeamRaySamples",100,true),
                                             cfg.read_bool  (m_name, "filterRanges", true, true),
                                             cfg.read_double(m_name, "filterRanges_minDelta", 0.05, true),
                                             cfg.read_int   (m_name, "nThetaSamples", 30,true),
                                             cfg.read_double(m_name, "quantification", 0.2,true),
                                             (BEAM_POINT_GENERATION) cfg.read_int(m_name, "beamSamplingMethod", true));

    vector<float> sensorPose, sensorPoseCovDiag;
    cfg.read_vector<vector<float>>(m_name, "sensorPoseOnRobot",vector<float>(),sensorPose, true);
    cfg.read_vector<vector<float>>(m_name, "sensorPoseOnRobot_covDiag",vector<float>(),sensorPoseCovDiag, true);

    CMatrixDouble66 sensorPose_cov = Eigen::Matrix<double,6,6>::Zero();
    for(int i = 0; i < 6; i++)
        sensorPose_cov(i,i) = sensorPoseCovDiag[i] + 1e-6;
    m_sensorPoseOnRobot = CPose3DPDFGaussian(CPose3D(sensorPose[0], sensorPose[1], sensorPose[2],
                                             DEG2RAD(sensorPose[3]), DEG2RAD(sensorPose[4]), DEG2RAD(sensorPose[5])),
                                             sensorPose_cov);
}

vector<sonarMeasure> observationSimulator_sonar::getFullSensorPlaneRawMeasures(const CPose3D& robotGlobalPose,
                                                                               int nPts) const
{
    vector<sonarMeasure> rawMeasures;

    CPose3D curGlobalSonarPose =  robotGlobalPose + m_sensorPoseOnRobot.mean, curBeamPose;
    Eigen::Vector3f source = Eigen::Vector3f::Zero();
    source << curGlobalSonarPose.x(), curGlobalSonarPose.y(), curGlobalSonarPose.z();

    double angleStep_rad = 2.*M_PI/(double)nPts, curAngle = 0.;
    
    igl::Hit hit;
    bool hasHit;
    sonarMeasure curMeas;
    for(int i = 0; i < nPts; i++)
    {
        curAngle += angleStep_rad;
        curBeamPose.setYawPitchRoll(curAngle ,0., 0.);

        hasHit = m_embreeIntersector->intersectRay(source,
                                                   getRayDir(curGlobalSonarPose + curBeamPose, curAngle),
                                                   hit);
        if(hasHit)
        {
            curMeas.range = hit.t;
            curMeas.phi   = curAngle;
            rawMeasures.push_back(curMeas);
        }
        else
            MRPT_LOG_ERROR_STREAM("[observationSimulator_sonar::getFullSensorPlaneRay] tracing failed");
    }

    return rawMeasures;
}

vector<CPointPDFGaussian> observationSimulator_sonar::getFullSensorPlaneGlobalMeasures(const CPose3D& robotGlobalPose,
                                                                                       int nPts) const
{
    vector<CPointPDFGaussian> globalMeas;
    vector<sonarMeasure> rawMeasures = getFullSensorPlaneRawMeasures(robotGlobalPose, nPts);

    globalMeas.reserve(rawMeasures.size());
    for(const sonarMeasure& meas : rawMeasures)
        globalMeas.push_back(fromSonarLocalCoordsToGlobalCoords(meas,
                                                                     robotGlobalPose,
                                                                     m_sensorPoseOnRobot.mean,
                                                                     m_characteristics.sphericalLocalCovariance));

    return globalMeas;
}

vector<CPointPDFGaussian> observationSimulator_sonar::getFullSensorPlaneLocalMeasures(const CPose3D& robotGlobalPose,
                                                                                      int nPts) const
{
    vector<CPointPDFGaussian> localMeas;
    vector<sonarMeasure> rawMeasures = getFullSensorPlaneRawMeasures(robotGlobalPose, nPts);

    localMeas.reserve(rawMeasures.size());
    for(const sonarMeasure& meas : rawMeasures)
        localMeas.push_back(fromSonarLocalCoordsToRobotLocalCoords(meas,
                                                                   m_sensorPoseOnRobot.mean,
                                                                   m_characteristics.sphericalLocalCovariance));

    return localMeas;
}

TPoint3D observationSimulator_sonar::sectionBarycenter(const CPose3D& robotGlobalPose) const
{   
    // Measured 3DP points are expressed in the sonar frame
    vector<CPointPDFGaussian> sectionPoints = getFullSensorPlaneLocalMeasures(robotGlobalPose, 720 /*nPts*/);
    CPoint3D barycenter;
    for(const CPointPDFGaussian& pt : sectionPoints)
    {
        barycenter.x_incr(pt.mean.x());
        barycenter.y_incr(pt.mean.y());
        barycenter.z_incr(pt.mean.z());
    }
    double inv_nSectionPoints = 1./(double)sectionPoints.size();

    // Return the barycenter expressed 
    return TPoint3D(barycenter.x() * inv_nSectionPoints,
                    barycenter.y() * inv_nSectionPoints,
                    barycenter.z() * inv_nSectionPoints);
}

Eigen::Vector3f observationSimulator_sonar::getRayDir(const CPose3D& pose,
                                                      const double yaw) const
{
    const Eigen::Matrix3d& R = pose.getRotationMatrix();
    Eigen::Vector3f dir = R.block<3,1>(0,0).template cast<float>();

    // When yaw is near Pi and the direction should be +z, we may have the direction became -z
    // Indeed, a rotation of +pi around +z is the same as a -pi rotation around -z !
    if(fabs(yaw - (-M_PI)) < 1e-5 &&
       fabs(dir(0) - 0.f) < 1e-5 &&
       fabs(dir(1) - 0.f) < 1e-5 &&
       fabs(dir(2) - (-1.f)) < 1e-5) // dir = -z
         dir *= -1;
    return dir;

}

ray observationSimulator_sonar::getRay_noBeamWidth(const Eigen::Vector3f &source,
                                                   const CPose3DPDFGaussian &beamGlobalPose,
                                                   const double yaw) const
{
    ray r;
    r.source = source;
    r.dir    = getRayDir(beamGlobalPose.mean, yaw);

    return r;
}

ray observationSimulator_sonar::getRay_pitch(const Eigen::Vector3f& source,
                                       const mrpt::poses::CPose3DPDFGaussian& beamGlobalPose,
                                       const double pitch,
                                       const double yaw)
{
    ray r;
    r.source = source;

    // Get a random pitch angle
    r.theta = pitch;
    CPose3D pitch_rotation(0,0,0,0,pitch,0);
    CPose3D rayGlobalPose = beamGlobalPose.mean + pitch_rotation;

    r.dir = getRayDir(rayGlobalPose, yaw);

    return r;
}

ray observationSimulator_sonar::getRay_oneRandomSample(const Eigen::Vector3f& source,
                                                       const mrpt::poses::CPose3DPDFGaussian& beamGlobalPose,
                                                       const double yaw)
{
    double theta = DEG2RAD((m_dis_theta(m_gen)*0.001 - 0.5)*m_characteristics.beamWidth);
    return getRay_pitch(source,
                        beamGlobalPose,
                        theta,
                        yaw);
}

vector<ray> observationSimulator_sonar::getRays(const CPose3DPDFGaussian &robotGlobalPose,
                                                const CPose3DPDFGaussian &sonar_angular_pose)
{
    vector<ray> rays;

    CPose3DPDFGaussian curGlobalPoseBeam =  robotGlobalPose + m_sensorPoseOnRobot + sonar_angular_pose;
    Eigen::Vector3f source = Eigen::Vector3f::Zero();
    source << curGlobalPoseBeam.mean.x(), curGlobalPoseBeam.mean.y(), curGlobalPoseBeam.mean.z();

    if(m_characteristics.ignoreBeamWidth)
    {
        // In this case, the beam is the ray !
        rays.push_back(getRay_noBeamWidth(source,
                                          curGlobalPoseBeam,
                                          sonar_angular_pose.mean.yaw()));
    }
    else
    {
        switch(m_characteristics.beamSamplingMethod)
        {
        case ONE_RANDOM_SAMPLE:
        {
            rays.push_back(getRay_oneRandomSample(source,
                                                  curGlobalPoseBeam,
                                                  sonar_angular_pose.mean.yaw()));
            break;
        }
        case FIXED_SAMPLES:
        {
            double step = m_characteristics.beamWidth/(double)m_characteristics.nBeamRaySamples, pitch = -0.5*m_characteristics.beamWidth;
            for(int i = 0; i < m_characteristics.nBeamRaySamples; i++)
            {
                rays.push_back(getRay_pitch(source,
                                            curGlobalPoseBeam,
                                            DEG2RAD(pitch),
                                            sonar_angular_pose.mean.yaw()
                                                  ));
                pitch += step;
            }     
            break;
        }
        case SAMPLE_AT_CENTER:
        {
            rays.push_back(getRay_pitch(source,
                                        curGlobalPoseBeam,
                                        0.,
                                        sonar_angular_pose.mean.yaw()));
            break;
        }
        default:
        {
            MRPT_LOG_ERROR_STREAM("Unknown beam sampling method !!!!");
            break;
        }
        }
    }

    return rays;
}

double observationSimulator_sonar::rangeQuantification(const double range) const
{
    if(m_characteristics.quantification_range <= 0)
        return range;
    else
    {
        // Round to the nearest quantification
        double k = floor(range/m_characteristics.quantification_range);
        double lowBound = k*m_characteristics.quantification_range, 
               highBound =  (k+1)*m_characteristics.quantification_range;
        double a1 = range - lowBound, a2 = highBound - range;

        return (a1 > a2) ? highBound : lowBound;
    }
}

vector<sonarMeasure> observationSimulator_sonar::simulateObservation(const CPose3DPDFGaussian &curGlobalPose,
                                                                     const CPose3DPDFGaussian &sonar_rotation)
{
    vector<sonarMeasure> measures;
    vector<ray> raysToIntersect = getRays(curGlobalPose, sonar_rotation);

    igl::Hit hit;
    bool hasHit;
    sonarMeasure measure;
    for(const ray& curRay : raysToIntersect)
    {
        hasHit = m_embreeIntersector->intersectRay(curRay.source,
                                                   curRay.dir,
                                                   hit);

        if(hasHit && hit.t <= m_characteristics.maxRange)
        {
            measure.range = rangeQuantification(hit.t);
            measure.phi   = sonar_rotation.mean.yaw(); 
            measure.theta = curRay.theta;
            measures.push_back(measure);
        }
        /*else
        {
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
            cout << "!!!!!!!!!!!!!!!!!! Failed to intersect with mesh !!!!!!!!!!!!!!!!!!!!!! " << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
            cout << " ---> source  : " << curRay.source << endl;
            cout << " ---> dir     : " << curRay.dir    << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! " << endl;
        }*/
    }

    // Only keep measure with ranges separated enough
    if(m_characteristics.filterRanges)
        measures = filterRanges(measures);

    return measures;
}

int observationSimulator_sonar::getThetaIdx(double theta) const
{
    return (theta + 0.5*m_characteristics.beamWidth_rad + 1e-4)/(double)m_characteristics.stepThetaSample_rad;
}

vector<sonarMeasure> observationSimulator_sonar::filterRanges(const vector<sonarMeasure>& rawMeasures) const
{
    vector<sonarMeasure> filteredMeasures;
    if(!rawMeasures.empty())
    {
        if(rawMeasures.size() == 1)
            filteredMeasures.push_back(rawMeasures[0]);
        else
        {
            // Sort by increasing range
            filteredMeasures = rawMeasures;
            std::sort(filteredMeasures.begin(), filteredMeasures.end(), 
                      [](const sonarMeasure& a, const sonarMeasure& b){return a.range < b.range;});

            // Merge similar values
            vector<sonarMeasure> mergedMeasures;
            mergedMeasures.push_back(filteredMeasures[0]);
            double refRange = filteredMeasures[0].range;
            for(const sonarMeasure& sm : filteredMeasures)
            {
                if(fabs(sm.range - refRange) > m_characteristics.filterRanges_minDelta)
                {
                    mergedMeasures.push_back(sm);
                    refRange = sm.range;
                }
            }
            filteredMeasures = mergedMeasures;
        }
    }

    return filteredMeasures;
}
