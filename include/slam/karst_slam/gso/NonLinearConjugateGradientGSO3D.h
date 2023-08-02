#ifndef NONLINEARCONJUGATEGRADIENTGSO3D_H
#define NONLINEARCONJUGATEGRADIENTGSO3D_H

#include "BaseGtsamGSO3D.h"

// Forward declarations
namespace gtsam{class NonlinearOptimizerParams;}

namespace karst_slam{namespace gso{
DEFINE_PARAMETER_TYPE_SPE(NonLinearConjugateGradientGSO3D, gtsam::NonlinearOptimizerParams)
/** Graph slam optimization using Non Linear CG impl of gtsam */
class NonLinearConjugateGradientGSO3D : public BaseGtsamGSO3D<NonLinearConjugateGradientGSO3D>
{
public:
    NonLinearConjugateGradientGSO3D();
    explicit NonLinearConjugateGradientGSO3D(const std::string& configFile);
    explicit NonLinearConjugateGradientGSO3D(const mrpt::utils::CConfigFile& configFile);
    ~NonLinearConjugateGradientGSO3D(){}

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;

};
}} // end namespaces

#endif // NONLINEARCONJUGATEGRADIENTGSO3D_H
