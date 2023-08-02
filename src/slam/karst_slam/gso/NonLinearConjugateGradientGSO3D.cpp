#include "karst_slam/gso/NonLinearConjugateGradientGSO3D.h"
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include "mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h"
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

NonLinearConjugateGradientGSO3D::NonLinearConjugateGradientGSO3D():
    BaseGtsamGSO3D<NonLinearConjugateGradientGSO3D>("NonLinearConjugateGradientGSO3D")
{}

NonLinearConjugateGradientGSO3D::NonLinearConjugateGradientGSO3D(const string& configFile) :
    BaseGtsamGSO3D<NonLinearConjugateGradientGSO3D>("NonLinearConjugateGradientGSO3D",configFile)
{}

NonLinearConjugateGradientGSO3D::NonLinearConjugateGradientGSO3D(const CConfigFile& configFile) :
    BaseGtsamGSO3D<NonLinearConjugateGradientGSO3D>("NonLinearConjugateGradientGSO3D",configFile)
{}

void NonLinearConjugateGradientGSO3D::loadOptimizerParams_(const CConfigFile& source)
{
    convertToNonLinearOptimizerParams(source,"OptimizerParameters",m_params);
}

void NonLinearConjugateGradientGSO3D::printParams_() const
{
    printNonLinearParams();
    cout << "------- [CDoglegGSO Parameters] ---------" << endl;
    cout << endl;
}

gtsam::Values NonLinearConjugateGradientGSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    // Create the optimizer and optimize
    gtsam::NonlinearConjugateGradientOptimizer optimizer(graph.getFactorGraph(), graph.getValues(), m_params);
    return optimizer.optimize();
}
