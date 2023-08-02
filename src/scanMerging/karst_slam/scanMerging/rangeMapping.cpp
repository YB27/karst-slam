#include "karst_slam/scanMerging/rangeMapping.h"

using namespace std;
using namespace mrpt::utils;
using namespace karst_slam::scanMerging;

function<double(double)> rangeMapping::rangeMap;
function<double(double)> rangeMapping::inverseRangeMap;

void rangeMapping::configure(const CConfigFile& cfg)
{
    string rangeMapFunction = cfg.read_string("GaussianProcess", "rangeMapFunction", "None", true);
    if(rangeMapFunction == "None")
    {
        inverseRangeMap = [](double d)->double {return d;};
        rangeMap = [](double d)->double {return d;};
    }
    else if(rangeMapFunction == "sqrt")
    {
        inverseRangeMap = [](double d)->double {return d*d;};
        rangeMap = [](double d)->double {return sqrt(d);};
    }
    else if(rangeMapFunction == "ln")
    {
        inverseRangeMap = [](double d)->double {return exp(d);};
        rangeMap = [](double d)->double {return log(d);};
    }
}
