#include "karst_slam/gso/DoglegGSO3D.h"
#include "mrpt-gtsam/wrapperGTSAM/conversionOptimizerParams.h"
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <mrpt/utils/CConfigFile.h>

using namespace std;
using namespace mrpt::utils;
using namespace mrpt_gtsam::wrapperGTSAM;
using namespace karst_slam;
using namespace karst_slam::gso;

DoglegGSO3D::DoglegGSO3D() :
    BaseGtsamGSO3D<DoglegGSO3D>("DoglegGSO3D")
{}

DoglegGSO3D::DoglegGSO3D(const string& configFile) :
    BaseGtsamGSO3D<DoglegGSO3D>("DoglegGSO3D",configFile)
{}

DoglegGSO3D::DoglegGSO3D(const CConfigFile& configFile) :
    BaseGtsamGSO3D<DoglegGSO3D>("DoglegGSO3D",configFile)
{}

void DoglegGSO3D::loadOptimizerParams_(const CConfigFile& source)
{
    convertToDoglegOptimizerParams(source,"OptimizerParameters", m_params);
}

void DoglegGSO3D::printParams_() const
{
    printNonLinearParams();
    cout << "------- [CDoglegGSO Parameters] ---------" << endl;
    cout << "deltaInitial  = " << m_params.deltaInitial << endl;
    cout << "verbosityDL   = " << gtsamDLVerbosityValue2Name.at(m_params.verbosityDL) << endl;
    cout << endl;
}

gtsam::Values DoglegGSO3D::execOptimization_(const gtsamGraph& graph,const set<TNodeID>* nodes_to_optimize)
{
    // Create the optimizer and optimize
    gtsam::DoglegOptimizer optimizer(graph.getFactorGraph(), graph.getValues(), m_params);
    return optimizer.optimize();
}
