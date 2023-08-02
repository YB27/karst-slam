#include "karst_slam/gso/GaussNewtonGSO3D.h"
#include "mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

GaussNewtonGSO3D::GaussNewtonGSO3D() :
    BaseGtsamGSO3D<GaussNewtonGSO3D>("GaussNewtonGSO3D")
{}

GaussNewtonGSO3D::GaussNewtonGSO3D(const string& configFile) :
    BaseGtsamGSO3D<GaussNewtonGSO3D>("GaussNewtonGSO3D", configFile)
{}

GaussNewtonGSO3D::GaussNewtonGSO3D(const CConfigFile& configFile) :
    BaseGtsamGSO3D<GaussNewtonGSO3D>("GaussNewtonGSO3D", configFile)
{}

void GaussNewtonGSO3D::loadOptimizerParams_(const CConfigFile& source)
{
   convertToNonLinearOptimizerParams(source,"OptimizerParameters",this->m_params);
}

void GaussNewtonGSO3D::printParams_() const
{
    printNonLinearParams();
    cout << "------- [CGaussNewtonGSO Parameters] ---------" << endl;
    cout << endl;
}

gtsam::Values GaussNewtonGSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    // Create the optimizer and optimize

    // Debug //
    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    graph.print();
    cout << "----------------------------"<<endl;
    cout << "Total error : " << graph.error() << endl;
    cout << "----------------------------"<<endl;
    graph.printErrorsPerFactor();
    cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<endl;
    //-------//

    gtsam::GaussNewtonOptimizer optimizer(graph.getFactorGraph(), graph.getValues(), m_params);
    return optimizer.optimize();
}


