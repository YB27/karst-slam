#ifndef ISAMGSO3D_H
#define ISAMGSO3D_H

#include "BaseGtsamIsamGSO3D.h"

// Forward declarations
namespace gtsam{class NonlinearISAM;}

namespace karst_slam{namespace gso{
/** Graph slam optimization using ISAM impl of gtsam */
class IsamGSO3D : public BaseGtsamIsamGSO3D
{
public:
    using params_t = int;

    IsamGSO3D();
    explicit IsamGSO3D(const std::string& configFile);
    explicit IsamGSO3D(const mrpt::utils::CConfigFile& configFile);
    IsamGSO3D(const std::string& configFile, int reorderInterval);
    ~IsamGSO3D(){}

    bool checkInit_()const override;

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;
    int m_reorderInterval;

private:
    std::shared_ptr<gtsam::NonlinearISAM> m_nonlinearIsam;
};
}} // end namespaces

#endif // ISAMGSO3D_H
