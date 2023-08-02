#include "karst_slam/gso/IsamGSO3D.h"
#include <gtsam/nonlinear/NonlinearISAM.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

IsamGSO3D::IsamGSO3D() :
    BaseGtsamIsamGSO3D("IsamGSO3D"),
    m_nonlinearIsam(make_shared<gtsam::NonlinearISAM>())
{}

IsamGSO3D::IsamGSO3D(const CConfigFile& configFile) : BaseGtsamIsamGSO3D("IsamGSO3D",configFile)
{
    loadOptimizerParams_(configFile);
    m_nonlinearIsam = make_shared<gtsam::NonlinearISAM>(m_reorderInterval);
    printParams_();
}

IsamGSO3D::IsamGSO3D(const string& configFile) :
    IsamGSO3D(CConfigFile(configFile))
{}

IsamGSO3D::IsamGSO3D(const string& configFile,
                    int reorderInterval) : BaseGtsamIsamGSO3D("IsamGSO3D",configFile),
                                                   m_reorderInterval(reorderInterval)
{
    m_nonlinearIsam = make_shared<gtsam::NonlinearISAM>(m_reorderInterval);//gtsam::NonlinearISAM(m_reorderInterval);
    printParams_();
}

bool IsamGSO3D::checkInit_()const
{
    return (m_nonlinearIsam != nullptr);
}

void IsamGSO3D::loadOptimizerParams_(const CConfigFile& source)
{
    convertToISAMParams(source, "OptimizerParameters", m_reorderInterval);
}

void IsamGSO3D::printParams_() const
{
    cout << "--------- [CISAMGSO Parameters ] ---------" << endl;
    cout << "reorderInterval = " << m_reorderInterval    << endl;
    cout << endl;
}

gtsam::Values IsamGSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    cout << "In execOptimization_ ..." <<  endl;

    gtsam::Values est;
    //try{
    // ISAM only needs the new added edges -> do not care about nodes_to_optimize here
    // Update ISAM with the new factors

    //cout << "[CIsamGSO --> m_graphConverter.getLastUpdateInfo()]" << endl;
    const updateInfo& lastUpdateInfo = m_pose_graph->getConvertedGraphLastUpdateInfo();//m_graphConverter.getLastUpdateInfo();

    cout << "lastUpdateInfo : "<< endl;
    for(const auto& f : lastUpdateInfo.newFactors)
        f->print();
    lastUpdateInfo.newValues.print();

    //cout << "[CIsamGSO --> updateGraph()]" << endl;
    gtsam::NonlinearFactorGraph updateGraph(lastUpdateInfo.newFactors);

    //cout << "[CIsamGSO --> update()]" << endl;
    m_nonlinearIsam->update(updateGraph, lastUpdateInfo.newValues);

    est = m_nonlinearIsam->estimate();
    //cout << "[CIsamGSO --> return estimate] : ";
    //est.print();
    //}
//    catch(const tbb::tbb_exception& e)
//    {
//        cout << e.what() << endl;
//    }
    return est;
}
