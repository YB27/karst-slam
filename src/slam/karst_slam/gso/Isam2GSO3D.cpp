#include "karst_slam/gso/Isam2GSO3D.h"
#include <gtsam/nonlinear/ISAM2.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

Isam2GSO3D::Isam2GSO3D() :
    BaseGtsamIsamGSO3D("Isam2GSO3D"),
    m_nonlinearIsam2(make_shared<gtsam::ISAM2>())
{}

Isam2GSO3D::Isam2GSO3D(const CConfigFile &configFile) : BaseGtsamIsamGSO3D("Isam2GSO3D",configFile)
{
    loadOptimizerParams_(configFile);
    m_nonlinearIsam2 = make_shared<gtsam::ISAM2>(m_params);
    printParams_();
}

Isam2GSO3D::Isam2GSO3D(const string& configFile) :
    Isam2GSO3D(CConfigFile(configFile))
{}

bool Isam2GSO3D::checkInit_()const
{
    return (m_nonlinearIsam2 != nullptr);
}

void Isam2GSO3D::loadOptimizerParams_(const CConfigFile& source)
{
    convertToISAM2Params(source,"OptimizerParameters",m_params);
}

void Isam2GSO3D::printParams_() const
{
    cout << "------- [CISAM2GSO Parameters] ---------" << endl;
    cout << "relinearizeSkip                     = " << m_params.relinearizeSkip << std::endl;
    //std::cout << "relinearizeThreshold                = " << m_params.relinearizeThreshold  << std::endl;
    cout << "enableRelinearization               = " << m_params.enableRelinearization  << std::endl;
    cout << "evaluateNonlinearError              = " << m_params.evaluateNonlinearError  << endl;
    cout << "factorization                       = " << gtsamISAMFactorisationValue2Name.at(m_params.factorization)  << endl;
    cout << "cacheLinearizedFactors              = " << m_params.cacheLinearizedFactors  << endl;
    cout << "enableDetailedResults               = " << m_params.enableDetailedResults  << endl;
    cout << "enablePartialRelinearizationCheck   = " << m_params.enablePartialRelinearizationCheck  << endl;
    cout << "findUnusedFactorSlots               = " << m_params.findUnusedFactorSlots  << endl;
    cout << endl;
}

gtsam::Values Isam2GSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    cout << "In execOptimization_ ..." << endl;

    // ISAM only needs the new added edges -> do not care about nodes_to_optimize here
    // Update ISAM with the new factors
    const updateInfo& lastUpdateInfo = m_pose_graph->getConvertedGraphLastUpdateInfo();//m_graphConverter.getLastUpdateInfo();

    gtsam::NonlinearFactorGraph updateGraph(lastUpdateInfo.newFactors);
    m_nonlinearIsam2->update(updateGraph, lastUpdateInfo.newValues);

    // Several iterations of the non linear solver (Accuracy vs speed)
    for(int i = 0; i < m_nonlinearSolverIterations - 1;i++)
        m_nonlinearIsam2->update();

    return m_nonlinearIsam2->calculateEstimate();
}
