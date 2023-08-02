#include "karst_slam/gironaDataSetLoader/depthLogData.h"

using namespace std;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

void depthData::readFromLine_(stringstream &ss)
{
    string valStr;

    // Depth
    getline(ss, valStr, ',');
    depth = stod(valStr);
}

mrpt::obs::CObservationPtr depthData::convertToCObservation()
{
    ObservationDepthPtr obs = ObservationDepth::Create();

    obs->timestamp   = std::stoull(stamp); // TTimestamp is uint64 and unsigned long long has at least a size of 64
    obs->sensorLabel = "Depth sensor";
    obs->depth       = depth;
    obs->depth_std   = 0.0001; // Not exact but should be very small
    // As given in Table 2 of "Underwater caves sonar data set", A.Mallios et al.,IJRR 2017
    // Warning : In the paper it is given roll pitch yaw
    obs->sensorPose  = CPose3D(-0.06 , 0., -0.1, 0., 0., 0.);

    return obs;
}

void depthData::dumpToConsole_()
{
    cout << "++++++++ Depth ++++++++" << endl;
    cout << "Depth : " << depth << endl;
}
