#include "karst_slam/BaseParams.h"
#include <mrpt/utils/CStream.h>

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam;

void BaseParams::dumpToTextStream(CStream &out) const
{
    MRPT_START;
    out.printf("%s", getAsString().c_str());
    MRPT_END;
}

string BaseParams::getAsString() const
{
    MRPT_START;

    string str;
    getAsString(&str);
    return str;

    MRPT_END;
}
