#include "karst_slam/gironaDataSetLoader/relativeOdometryData.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

CObservationPtr odometry_relative::convertToCObservation()
{
    ObservationOdometryPtr obs = ObservationOdometry::Create();

    obs->timestamp   = std::stoull(stamp);
    obs->pose_pdf.mean.inverseComposeFrom(currentPose,prevPose);
    obs->pose_pdf.cov_inv = odometry_cov_matrix.inverse();
    obs->sensorLabel = "Relative Odometry given by dead reckoning EKF. To be used as reference.";

    return obs;
}
