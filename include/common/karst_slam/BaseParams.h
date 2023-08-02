#ifndef BASEPARAMS_H
#define BASEPARAMS_H

#include "internal.h"
#include <mrpt/utils/CLoadableOptions.h>

namespace karst_slam {

/** Base class for parameters following MRPT format */ 
struct BaseParams : public mrpt::utils::CLoadableOptions
{
    virtual void loadFromConfigFile(const mrpt::utils::CConfigFileBase &source,
                                    const std::string &section) = 0;

    /** Return a string with the configuration parameters */
    virtual void getAsString(std::string* params_out) const = 0;
    std::string getAsString() const;

    virtual void dumpToTextStream(mrpt::utils::CStream &out) const;
};

}// end namespace


#endif // BASEPARAMS_H
