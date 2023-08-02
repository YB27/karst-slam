#ifndef DOGLEGGSO3D_H
#define DOGLEGGSO3D_H

#include "BaseGtsamGSO3D.h"

// Forward declarations
namespace gtsam{class DoglegParams;}

namespace karst_slam{namespace gso{
DEFINE_PARAMETER_TYPE_SPE(DoglegGSO3D, gtsam::DoglegParams)

/** Graph slam optimization using DogLeg impl of gtsam */
class DoglegGSO3D : public BaseGtsamGSO3D<DoglegGSO3D>
{
public:
    DoglegGSO3D();
    explicit DoglegGSO3D(const std::string& configFile);
    explicit DoglegGSO3D(const mrpt::utils::CConfigFile& configFile);

    ~DoglegGSO3D(){}

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,
                                    const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;

};
}} // end namespaces
#endif // DOGLEGGSO3D_H
