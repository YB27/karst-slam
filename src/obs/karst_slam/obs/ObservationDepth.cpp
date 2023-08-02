#include "karst_slam/obs/ObservationDepth.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace karst_slam;
using namespace karst_slam::obs;

IMPLEMENTS_SERIALIZABLE(ObservationDepth, CObservation,karst_slam::obs)

void ObservationDepth::writeToStream(CStream &out, int *getVersion) const
{
    if(getVersion)
        *getVersion = 0;
    else
    {
        out << depth;
        out << depth_std;
        out << sensorLabel;
        out << timestamp;
    }
}

void ObservationDepth::readFromStream(CStream &in, int version)
{
    in >> depth;
    in >> depth_std;
    in >> sensorLabel;
    in >> timestamp;
}

void ObservationDepth::getDescriptionAsText(ostream &o) const
{
    CObservation::getDescriptionAsText(o);

    o << "Depth     : " << depth     << endl;
    o << "Depth std : " << depth_std << endl;
}
