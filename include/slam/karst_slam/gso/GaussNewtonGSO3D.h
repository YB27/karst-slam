#ifndef GAUSSNEWTONGSO3D_H
#define GAUSSNEWTONGSO3D_H

#include "BaseGtsamGSO3D.h"
//#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

// Forward declarations
namespace gtsam{class GaussNewtonParams;}

namespace karst_slam{namespace gso{
DEFINE_PARAMETER_TYPE_SPE(GaussNewtonGSO3D, gtsam::GaussNewtonParams)

/** Graph slam optimization using GaussNewton impl of gtsam */
class GaussNewtonGSO3D : public BaseGtsamGSO3D<GaussNewtonGSO3D>
{
public:
    GaussNewtonGSO3D();
    explicit GaussNewtonGSO3D(const std::string& configFile);
    explicit GaussNewtonGSO3D(const mrpt::utils::CConfigFile& configFile);

    ~GaussNewtonGSO3D(){}

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph, const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;

};
}} // end namespaces

#endif // GAUSSNEWTONGSO3D_H
