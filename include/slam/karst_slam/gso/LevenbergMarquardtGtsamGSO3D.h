#ifndef LEVENBERGMARQUARDTGTSAMGSO3D_H
#define LEVENBERGMARQUARDTGTSAMGSO3D_H

#include "BaseGtsamGSO3D.h"

// Forward declarations
namespace gtsam{class LevenbergMarquardtParams;}

namespace karst_slam{namespace gso{
DEFINE_PARAMETER_TYPE_SPE(LevenbergMarquardtGtsamGSO3D, gtsam::LevenbergMarquardtParams)

/** Graph slam optimization using LM impl of gtsam  */
class LevenbergMarquardtGtsamGSO3D : public BaseGtsamGSO3D<LevenbergMarquardtGtsamGSO3D>
{
public:
    LevenbergMarquardtGtsamGSO3D();
    explicit LevenbergMarquardtGtsamGSO3D(const std::string& configFile);
    explicit LevenbergMarquardtGtsamGSO3D(const mrpt::utils::CConfigFile& configFile);

    ~LevenbergMarquardtGtsamGSO3D(){}

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;

};
}} // end namespaces

#endif // LEVENBERGMARQUARDTGTSAMGSO3D_H
