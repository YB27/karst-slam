#include "karst_slam/obs/ObservationMSISBeam.h"
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationMSISBeam, CObservation,karst_slam::obs)

ObservationMSISBeamPtr ObservationMSISBeam::Create(const std::string& deviceName)
{
    return ObservationMSISBeamPtr(new ObservationMSISBeam(deviceName));
}


ObservationMSISBeam::ObservationMSISBeam(const string &deviceName)
{
    sensorLabel = deviceName;
    m_intensities = vector<int>();
}

void ObservationMSISBeam::writeToStream(CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        out << sensorLabel;
        out << m_angle;
        out << m_intensities;
        out << m_nAngularStep;
        out << m_maxRange;
        out << m_sonarPoseInBodyFrame;
        out << timestamp;
    }
}

void ObservationMSISBeam::readFromStream(CStream &in, int version)
{
    in >> sensorLabel;
    in >> m_angle;
    in >> m_intensities;
    in >> m_nAngularStep;
    in >> m_maxRange;
    in >> m_sonarPoseInBodyFrame;
    in >> timestamp;
}

void ObservationMSISBeam::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Beam from the device : " << sensorLabel << endl;
    o << "Angle : " << m_angle << endl;
    o << "Intensities : " << endl;
    o << "[";
    for(const int& v : m_intensities)
        o << v << ",";
    o << "]" << endl;
}
