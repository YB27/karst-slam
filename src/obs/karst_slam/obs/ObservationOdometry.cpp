#include "karst_slam/obs/ObservationOdometry.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationOdometry, CObservation,karst_slam::obs)

void ObservationOdometry::writeToStream(CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        out << pose_pdf;
        out << sensorLabel;
        out << timestamp;
    }
}

void ObservationOdometry::readFromStream(CStream &in, int version)
{
    in >> pose_pdf;
    in >> sensorLabel;
    in >> timestamp;
}

void ObservationOdometry::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Pose pdf : " << pose_pdf << endl;
}
