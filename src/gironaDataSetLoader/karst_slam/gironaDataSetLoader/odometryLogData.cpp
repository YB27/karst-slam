#include "karst_slam/gironaDataSetLoader/odometryLogData.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

void odometry::readFromLine_(stringstream& ss)
{
    string valStr;
    // Translation
    for(int i = 0; i < 3; i++)
    {
        getline(ss,valStr, ',');
        t[i] = stod(valStr);
    }

    // Rotation (represented by quaternion)
    for(int i = 0; i < 4; i++)
    {
        getline(ss,valStr, ',');
        q[i] = stod(valStr);
    }
}

CObservationPtr odometry::convertToCObservation()
{
    ObservationOdometryPtr obs = ObservationOdometry::Create();

    obs->timestamp       = std::stoull(stamp);
    // In MRPT, quaternion are [qw qx qy qz] while [qx qy qz qw] in the dataset logs
    obs->pose_pdf.mean   = CPose3D(mrpt::math::CQuaternionDouble(q[3], q[0], q[1], q[2]), t[0], t[1], t[2]);
    obs->sensorLabel     = "Odometry given by dead reckoning EKF. To be used as reference.";

    return obs;
}

void odometry::dumpToConsole_()
{
    cout << "++++++++ odometry ++++++++" << endl;
    cout << "t : ";
    for(int i = 0; i < 3; i++)
        cout << t[i] << " , ";
    cout << endl;
    cout << "q : ";
    for(int i = 0; i < 4; i++)
        cout << q[i] << " , ";
    cout << endl;
}
