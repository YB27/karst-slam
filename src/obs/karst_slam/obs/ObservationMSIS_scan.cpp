#include "karst_slam/obs/ObservationMSIS_scan.h"
#include <mrpt/utils/CStream.h>
#include <mrpt/obs/CObservation3DRangeScan.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationMSIS_scan, CObservation, karst_slam::obs)

ObservationMSIS_scanPtr ObservationMSIS_scan::Create(beams_t&& beamMap, int nPoints_,
                                                     const CPose3DPDFGaussian &refFrameGlobalPoseAtCreation_,
                                                     //const CPose3DPDFGaussian& refFrameLocalPose_,
                                                     const CPose3DPDFGaussian &refFrameToEndFramePose_,
                                                     const string& deviceName,
                                                     scanID scanId)
{
    return ObservationMSIS_scanPtr(new ObservationMSIS_scan(move(beamMap),
                                                            nPoints_,
                                                            refFrameGlobalPoseAtCreation_,
                                                            //refFrameLocalPose_ ,
                                                            refFrameToEndFramePose_,
                                                            deviceName,
                                                            scanId));
}

ObservationMSIS_scan::ObservationMSIS_scan(beams_t&& beamMap, int nPoints_,
                                           const CPose3DPDFGaussian& refFrameGlobalPoseAtCreation_,
                                          // const CPose3DPDFGaussian &refFrameLocalPose_,
                                           const CPose3DPDFGaussian& refFrameToEndFramePose_,
                                           const string& deviceName,
                                           scanID scanId)
{
//    cout << "[observationMSIS_scan::observationMSIS_scan] Create observationMSIS_scan with reference frame pose : ";
//    cout << refFrameGlobalPoseAtCreation_ << endl;
    sensorLabel = deviceName;
    m_nPoints = nPoints_;
    m_refFrameGlobalPoseAtCreation = refFrameGlobalPoseAtCreation_;
    //m_refFrameLocalPose = refFrameLocalPose_;
    m_refFrameToEndFramePose = refFrameToEndFramePose_;
    m_scanId = scanId;

    // Reserve
    m_points.resize(4,nPoints_);
    m_cov.reserve(nPoints_);

    // Concatened all points in one matrix and all cov in one vector
    int lastCol = 0, curBeam_nPoints;
    for(const auto& pair : beamMap)
    {
        const points_mat_t& curBeamPts = pair.second.points;
        curBeam_nPoints = curBeamPts.cols();
        // ToDo : More efficient way ?
        m_points.block(0,lastCol,4,curBeam_nPoints) = curBeamPts;
        lastCol += curBeam_nPoints;

        const eigenAlignedVector<cov_t>& covs = pair.second.cov;
        m_cov.insert(m_cov.end(),
                   std::make_move_iterator(covs.begin()),
                   std::make_move_iterator(covs.end()));
    }

    // Clear the beam points ?
    beamMap.clear();
}

void ObservationMSIS_scan::composeWithPose(const CPose3D &pose)
{
    // Points pose
    const mrpt::math::CMatrixDouble44& T = pose.getHomogeneousMatrixVal();
    const mrpt::math::CMatrixDouble33& R = pose.getRotationMatrix();
    m_points  = T*m_points;

    // Covariance
    for(eigenAlignedVector<cov_t>::iterator it_cov = m_cov.begin(); it_cov != m_cov.end();it_cov++)
       R.multiply_HCHt(*it_cov, *it_cov, false);
}

ObservationMSIS_scanPtr ObservationMSIS_scan::createComposeWithPose(const mrpt::poses::CPose3D& pose) const
{
    ObservationMSIS_scanPtr composedScan = ObservationMSIS_scan::Create();

    // Points pose
    const mrpt::math::CMatrixDouble44& T = pose.getHomogeneousMatrixVal();
    const mrpt::math::CMatrixDouble33& R = pose.getRotationMatrix();
    composedScan->m_points  = T*m_points;

    composedScan->m_nPoints = composedScan->m_points.cols();

    // Covariance
    composedScan->m_cov.resize(composedScan->m_nPoints);
    eigenAlignedVector<cov_t>::iterator it_comp_cov = composedScan->m_cov.begin();
    for(eigenAlignedVector<cov_t>::const_iterator it_cov = m_cov.begin(); it_cov != m_cov.end();it_cov++)
    {
        R.multiply_HCHt(*it_cov, *it_comp_cov, false);
        it_comp_cov++;
    }

    return composedScan;
}

CObservation3DRangeScan ObservationMSIS_scan::convertTo3DRangeScan() const
{
    CObservation3DRangeScan rangeScan;
    rangeScan.hasRangeImage = false;
    rangeScan.hasIntensityImage = false;
    rangeScan.hasConfidenceImage = false;
    rangeScan.hasPoints3D = true;

    rangeScan.points3D_x.reserve(m_nPoints);
    rangeScan.points3D_y.reserve(m_nPoints);
    rangeScan.points3D_z.reserve(m_nPoints);
    for(int i = 0; i < m_nPoints; i++)
    {
        rangeScan.points3D_x.push_back(m_points(i,0));
        rangeScan.points3D_y.push_back(m_points(i,1));
        rangeScan.points3D_z.push_back(m_points(i,2));
    }

    return rangeScan;
}

void ObservationMSIS_scan::writeToStream(mrpt::utils::CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        cout << "[observationMSIS_scan::writeToStream] Not correctly implemented yet !" << endl;
        out << timestamp;
        out << m_refFrameGlobalPoseAtCreation;
        out << m_nPoints;
        out << sensorLabel;
    }
}

void ObservationMSIS_scan::readFromStream(mrpt::utils::CStream &in, int version)
{
    cout << "[observationMSIS_scan::readFromStream] Not correctly implemented yet !" << endl;

    in >> timestamp;
    in >> m_refFrameGlobalPoseAtCreation;
    in >> m_nPoints;
    in >> sensorLabel;
}

void ObservationMSIS_scan::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Scan from the device : " << sensorLabel << endl;
    o << "Reference frame : " << m_refFrameGlobalPoseAtCreation << endl;
    o << "#Point : " << m_nPoints << endl;
}

void ObservationMSIS_scan::printScanData() const
{
   cout << "MSIS scan data : " << endl;
   for(int c = 0; c < m_points.cols(); c++)
   {
       cout << "----> Point " << c << endl;
       cout << "Point pose : ";
       for(int r = 0; r < m_points.rows(); r++)
           cout << m_points(r,c) << " , ";

       cout << endl;
       cout << "Covariance : " << endl;
       cout << m_cov[c] << endl;
       cout << "-----------------" << endl;
   }
}
