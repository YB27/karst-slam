#ifndef ISAM2GSO3D_H
#define ISAM2GSO3D_H

#include "BaseGtsamIsamGSO3D.h"

// Forward declarations
namespace gtsam{class ISAM2;}

namespace karst_slam{namespace gso{
/* *Graph slam optimization using ISAM2 impl of gtsam  */ 
class Isam2GSO3D : public BaseGtsamIsamGSO3D
{
public:
    Isam2GSO3D();
    explicit Isam2GSO3D(const std::string& configFile);
    explicit Isam2GSO3D(const mrpt::utils::CConfigFile& configFile);
    ~Isam2GSO3D(){}

    inline int getNLSolverIterations() const {return m_nonlinearSolverIterations;}
    inline void setNLSolverIterations(int iter){m_nonlinearSolverIterations = iter;}

    bool checkInit_()const override;

protected:
    inline void loadOptimizerParams_(const mrpt::utils::CConfigFile& source);
    void printParams_() const;

    gtsam::Values execOptimization_(const mrpt_gtsam::wrapperGTSAM::gtsamGraph& graph,const std::set<mrpt::utils::TNodeID>* nodes_to_optimize = nullptr) override;

private:
    std::shared_ptr<gtsam::ISAM2> m_nonlinearIsam2;
    int m_nonlinearSolverIterations = 1;
};
}} // end namespaces

#endif //ISAM2GSO3D_H
