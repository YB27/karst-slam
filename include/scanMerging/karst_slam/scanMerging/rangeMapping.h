#ifndef DEF_RANGEMAPPING_H
#define DEF_RANGEMAPPING_H

#include <mrpt/utils/CConfigFile.h>
#include <functional>
#include <cmath>

namespace karst_slam { namespace scanMerging{

/**
 * Struct used to define a map function for range measure from the sonar
 * This is used to assure positive value for range by setting for instance range = e^(gamma)
 */ 
struct rangeMapping
{
    rangeMapping() = default;

    static void configure(const mrpt::utils::CConfigFile& cfg);

    static std::function<double(double)> rangeMap;
    static std::function<double(double)> inverseRangeMap;
};

}}

#endif // DEF_RANGEMAPPING_H
