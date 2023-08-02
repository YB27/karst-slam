#include "karst_slam/obs/ObservationDVL.h"
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationDVL, CObservation, karst_slam::obs)

void ObservationDVL::writeToStream(mrpt::utils::CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        out << m_sensorPose
            << m_vx
            << m_vy
            << m_vz
            << timestamp
            << sensorLabel;
        out << m_relativeVelocityAccuracy
            << m_absoluteVelocityAccuracy;
    }
}

void ObservationDVL::readFromStream(mrpt::utils::CStream &in, int version)
{
    in >> m_sensorPose;
    in >> m_vx;
    in >> m_vy;
    in >> m_vz;
    in >> timestamp;
    in >> sensorLabel;
    in >> m_relativeVelocityAccuracy;
    in >> m_absoluteVelocityAccuracy;
}

void ObservationDVL::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Homogeneous matrix for the sensor's 3D pose, relative to robot base:\n";
    o << m_sensorPose.getHomogeneousMatrixVal() << m_sensorPose << endl;

    o << "Relative Velocity Accuracy : " << m_relativeVelocityAccuracy << " %\n";
    o << "Absolute Velocity Accuracy : " << m_absoluteVelocityAccuracy << "\n";

    o << "Measured Velocity (Accuracy) : \n";
    o << "  -> Vx : " << m_vx << " ("  << getAccuracy(m_vx) << ")\n";
    o << "  -> Vy : " << m_vy << " ("  << getAccuracy(m_vy) << ")\n";
    o << "  -> Vz : " << m_vz << " ("  << getAccuracy(m_vz) << ")\n";
}
