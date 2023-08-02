#include "karst_slam/sensors/MSIS_primary.h"
#include "karst_slam/sensors/Scan.h"
#include "karst_slam/obs/ObservationMSIS_scan.h"
#include "karst_slam/compare.h"
#include "karst_slam/tests/utils/testDefines.h"
#include <mrpt/utils/CConfigFile.h>

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <random>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::utils;
using namespace karst_slam::obs;
using namespace karst_slam::sensors;

class testableMSIS : public MSIS_primary
{
public:
    testableMSIS(const string& deviceName) : MSIS_primary(deviceName){}

    void setCurrentScan(const map<int, beamPoints>& scan)
    {
        m_currentScan   = scan;
        m_curScanPoints = 0;
        for(const auto& pair : scan)
            m_curScanPoints += pair.second.points.cols();
    }
    //void setRelativePoseToPrevBeam(const map<int, beamRelativePosePdf/*mrpt::poses::CPose3DPDFGaussian*/>& posePerBeam){m_relativePoseToPrevBeam = posePerBeam;}
    void setRelativePoseToPrevBeam(const vector<beamRelativePosePdf/*mrpt::poses::CPose3DPDFGaussian*/>& posePerBeam){m_relativePoseToPrevBeam = posePerBeam;}

    using MSIS_primary::fromIntensitiesToCartesianCoords;
    using MSIS_primary::generateFullScan;
};


bool equalScan(const ObservationMSIS_scan& m1, const ObservationMSIS_scan& m2)
{
    // check pose
    if(!equalPosePdf(m1.m_refFrameGlobalPoseAtCreation, m2.m_refFrameGlobalPoseAtCreation))
        return false;

//    if(!equalPosePdf(m1.m_refFrameLocalPose, m2.m_refFrameLocalPose))
//        return false;

    if(m1.m_nPoints != m2.m_nPoints)
        return false;

    if(m1.m_nPoints != m1.m_points.cols())
        return false;

    // Check covariances and points
    if(!equalMatrix(m1.m_points, m2.m_points))
        return false;
    for(int i = 0; i < m1.m_points.cols(); i++)
    {
        if(!equalMatrix(m1.m_cov[i], m2.m_cov[i]))
            return false;
    }

    return true;
}

vector<int> createRandomIntensities(int nbin, int intensityThreshold)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis(0, 2*intensityThreshold);

    vector<int> intensities(nbin);
    for(int& f : intensities)
        f = dis(gen);

    return intensities;
}

void printScan(const beamPoints& scan)
{
    int nPoint = scan.points.cols();
    for(int i = 0; i < nPoint; i++)
    {
        cout << "-------------" << endl;
        cout << "Point " << i  << " : " << scan.points(0,i) << "," << scan.points(1,i) << "," << scan.points(2,i) << endl;
        cout << "Cov Matrix : " << scan.cov[i] << endl;
    }
}

beamPoints generateScanPointsFromBeam(const vector<double>& ranges,
                                      double angle)
{
    beamPoints generatedScanPoints(ranges.size());
    double cosVal = cos(angle), sinVal = sin(angle);
    double cosVal_sqr = cosVal*cosVal, sinVal_sqr = sinVal*sinVal, sincosVal = sinVal*cosVal;
    int i = 0;
    double rr;
    for(const double& r : ranges)
    {
        rr = r*r;
        points_mat_t& pts = generatedScanPoints.points;
        pts(0,i) = r*cosVal;
        pts(1,i) = r*sinVal;
        pts(2,i) = 0.;
        pts(3,i) = 1;

        // Point covariance matrix is identity
        cov_t& cov = generatedScanPoints.cov[i];
        cov(0,0) = cosVal_sqr + rr*sinVal_sqr;
        cov(1,1) = sinVal_sqr + rr*cosVal_sqr;
        cov(0,1) = (1-rr)*sincosVal;
        i++;
    }

    return generatedScanPoints;
}

//int testMSIS_polarToCartesianCovariance()
//{
//    CartesianCov_lut cc_lut();

//    MSISParams params;
//    params.angle_yaw_Std = 0.;
//    params.angle_pitch_Std = 0.;
//    params.rangeStd = 1.;
//    params.verticalBeamWidth = 0.2f;
//    mrpt::math::CMatrixDouble22 sphericalCov;
//    sphericalCov(0,0) = 1.;
//    sphericalCov(1,1) = 0.;
//    sphericalCov(2,2) = 0.;

//    // Point with alpha = 0, r
//    double r = 5.6;
//{
//    cov_t cartesianCov_expected;
//    cartesianCov_expected << 1., 0. , 0.,
//                             0., 0. , 0.,
//                             0., 0. , K*r;
//    CartesianCov_lut cc_lut(sphericalCov,1.,0.,params.verticalBeamWidth);
//    cov_t cartesianCov_computed = cc_lut.getCovariance(r);
//    if(!equalMatrix(cartesianCov_expected, cartesianCov_computed))
//    {
//        FAILED
//        cout << "Expected cov : " << endl;
//        cout << cartesianCov_expected << endl;
//        cout << "Computed cov : " << endl;
//        cout << cartesianCov_computed << endl;
//        return -1;
//    }
//}
//{
//    // Point with alpha = 90 , r
//    cov_t cartesianCov_expected;
//    cartesianCov_expected << 0., 0. , 0.,
//                             0., 1. , 0.,
//                             0., 0. , K*r;
//    CartesianCov_lut cc_lut(sphericalCov,0.,1.,params.verticalBeamWidth);
//    cov_t cartesianCov_computed = cc_lut.getCovariance(r);
//    if(!equalMatrix(cartesianCov_expected, cartesianCov_computed))
//    {
//        FAILED
//        cout << "Expected cov : " << endl;
//        cout << cartesianCov_expected << endl;
//        cout << "Computed cov : " << endl;
//        cout << cartesianCov_computed << endl;
//        return -1;
//    }
//}

//    PASSED
//    return 0;
//}

int testMSIS_intensitiesToCartesianCoords()
{
    MSISParams params;
    params.intensityThreshold = 2;
    params.minDistanceFilter  = 3;
    params.nAngleStep = 10;
    params.nIntensityBins = 30;
    params.intensityStep = 0.5f;
    params.rangeStd = 1.f; // m
    params.angle_yaw_Std = 1.f; // deg
    params.angle_pitch_Std = 1.f;
    params.minIntensityBin = 2;

    testableMSIS msisTest("test");
    msisTest.setParams(params);

    // Data
    double angleStep = 2.*M_PI/(double)params.nAngleStep;
    vector<int> intensities = {0,3,1,0,1,3,5,5,5,5,3,1,0,0,0,0,1,4,5,4,1,0,0,0,0,5,4,6,5,0};
    int beamIdx = 2;
    double angle = (double)beamIdx*angleStep;
    vector<double> ranges;
    ranges.push_back(/*6.*/7.*params.intensityStep);
    ranges.push_back(/*18.*/19.*params.intensityStep);
    ranges.push_back(/*27.*/28.*params.intensityStep);

    // Expected values
    // ---> positions + covariances
    beamPoints expectedPoints = generateScanPointsFromBeam(ranges, angle);

    // Computed values
    beamPoints computedPoints = msisTest.fromIntensitiesToCartesianCoords(intensities, beamIdx);

    // Check only points here
    if(!equalMatrix(expectedPoints.points, computedPoints.points))
    {
        FAILED
        cout << "Expected Points : " << endl;
        printScan(expectedPoints);
        cout << "Computed Points : " << endl;
        printScan(computedPoints);
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }

}

int testMSIS_generateFullScan()
{
    MSISParams params;
    params.intensityThreshold = 2;
    params.minDistanceFilter  = 3;
    params.nAngleStep = 200;
    params.nIntensityBins = 15;
    params.intensityStep = 0.5f;
    params.rangeStd = 1.f;
    params.angle_yaw_Std = 0.f;
    params.angle_pitch_Std = 0.f;

    testableMSIS msisTest("test");
    msisTest.setParams(params);

    // Reference frame
    int center_idx = 0.5*(params.nAngleStep + 1);

    // generate poses
    mrpt::math::CMatrixDouble66 motionCov;
    motionCov(0,0) = 1.;
    double dx = 1.;
    double dx_center = center_idx*dx;
    mrpt::poses::CPose3DPDFGaussian curPos, increment,/*refFrameLocalPose,*/ refFrameToEndFrame;
    increment.mean = mrpt::poses::CPose3D(dx,0,0,0,0,0);
    increment.cov  = motionCov;
    std::map<int, mrpt::poses::CPose3DPDFGaussian> globalPosePerBeam;
    std::vector<beamRelativePosePdf> relativePoseToPrevBeam(params.nAngleStep);
    //std::map<int, beamRelativePosePdf> relativePoseToPrevBeam;

    mrpt::math::CMatrixDouble66 firstBeamGlobalCov;
    firstBeamGlobalCov(0,0) = 1.5;
    firstBeamGlobalCov(2,2) = 0.3;
    firstBeamGlobalCov(4,4) = 0.6;
    globalPosePerBeam[0]    = mrpt::poses::CPose3DPDFGaussian(mrpt::poses::CPose3D(1.,2.,3.,0.5*M_PI,0.,0.),
                                                                firstBeamGlobalCov);
    relativePoseToPrevBeam[0] = beamRelativePosePdf(mrpt::poses::CPose3DPDFGaussian(),true);//mrpt::poses::CPose3DPDFGaussian();
    for(int i = 1; i < params.nAngleStep; i++)
    {
        curPos += increment;
        globalPosePerBeam[i] = curPos;
        relativePoseToPrevBeam[i] = beamRelativePosePdf(increment,false);
//        if(i == center_idx)
//            refFrameLocalPose = curPos;
        if(i > center_idx)
            refFrameToEndFrame += increment;
    }

    // Generate the scan with motion
    int totalPts = 0;
    map<int, beamPoints> gt_beams, scan_beams;
    vector<int> intensities;
    beamPoints curBeamPoints;
    for(int i = 0; i < params.nAngleStep; i++)
    {
        intensities  = createRandomIntensities(params.nIntensityBins, params.intensityThreshold);

        // Set local points
        curBeamPoints = msisTest.fromIntensitiesToCartesianCoords(intensities, i);

        scan_beams[i] = curBeamPoints;

        // GT with motion
        // Express in refFrame
        // Cov ??
        beamPoints pointsInRefFrame(curBeamPoints);
        for(int j = 0; j < pointsInRefFrame.points.cols(); j++)
        {
            pointsInRefFrame.points(0,j) += i*dx - dx_center;
            pointsInRefFrame.cov[j](0,0) += fabs(1.*i - center_idx); // motion and point covariance indpt
        }
        gt_beams[i] = pointsInRefFrame;
        totalPts += pointsInRefFrame.points.cols();
    }

    // Create the final GT scan from the beams
    ObservationMSIS_scan gt_scan(move(gt_beams),
                                 totalPts,
                                 globalPosePerBeam[center_idx],
                                 //refFrameLocalPose,
                                 refFrameToEndFrame,
                                 "test",
                                 0);

    // Set and run scan correction
    msisTest.setCurrentScan(scan_beams);
    msisTest.setRelativePoseToPrevBeam(relativePoseToPrevBeam);
    msisTest.generateFullScan();
    ObservationMSIS_scanPtr computedScan = msisTest.getLastScan();

    if(!equalScan(*computedScan, gt_scan))
    {
        FAILED
        cout << "Expected scan : " << endl;
        MSIS::dumpScanToConsole(gt_scan);
        cout << endl;
        cout << "Computed scan : " << endl;
        MSIS::dumpScanToConsole(*computedScan);
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}
