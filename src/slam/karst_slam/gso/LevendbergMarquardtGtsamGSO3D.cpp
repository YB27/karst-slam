#include "karst_slam/gso/LevenbergMarquardtGtsamGSO3D.h"
#include "mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

LevenbergMarquardtGtsamGSO3D::LevenbergMarquardtGtsamGSO3D() :
    BaseGtsamGSO3D<LevenbergMarquardtGtsamGSO3D>("LevenbergMarquardtGtsamGSO3D")
{}

LevenbergMarquardtGtsamGSO3D::LevenbergMarquardtGtsamGSO3D(const string& configFile) :
    BaseGtsamGSO3D<LevenbergMarquardtGtsamGSO3D>("LevenbergMarquardtGtsamGSO3D",configFile)
{}

LevenbergMarquardtGtsamGSO3D::LevenbergMarquardtGtsamGSO3D(const CConfigFile& configFile) :
    BaseGtsamGSO3D<LevenbergMarquardtGtsamGSO3D>("LevenbergMarquardtGtsamGSO3D",configFile)
{}

void LevenbergMarquardtGtsamGSO3D::loadOptimizerParams_(const CConfigFile& source)
{
    convertToLevendbergMarquardtParams(source,"OptimizerParameters",m_params);
}

void LevenbergMarquardtGtsamGSO3D::printParams_() const
{
    printNonLinearParams();
    cout << "------- [CLevenbergMarquardtGtsamGSO Parameters] ---------"     << endl;
    cout << "lambdaInitial        = " << m_params.lambdaInitial        << endl;
    cout << "lambdaFactor         = " << m_params.lambdaFactor         << endl;
    cout << "lambdaUpperBound     = " << m_params.lambdaUpperBound     << endl;
    cout << "lambdaLowerBound     = " << m_params.lambdaLowerBound     << endl;
    cout << "minModelFidelity     = " << m_params.minModelFidelity     << endl;
    cout << "diagonalDamping      = " << m_params.diagonalDamping      << endl;
    cout << "useFixedLambdaFactor = " << m_params.useFixedLambdaFactor << endl;
    cout << "minDiagonal          = " << m_params.minDiagonal          << endl;
    cout << "maxDiagonal          = " << m_params.maxDiagonal          << endl;
    cout << "verbosityLM          = " << gtsamLMVerbosityValue2Name.at(m_params.verbosityLM) << endl;
}

gtsam::Values LevenbergMarquardtGtsamGSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    // Create the optimizer and optimize
    gtsam::LevenbergMarquardtOptimizer optimizer(graph.getFactorGraph(), graph.getValues(),m_params);
    return optimizer.optimize();
}
