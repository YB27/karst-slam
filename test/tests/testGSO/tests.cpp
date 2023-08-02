#include "testGSO.h"
#include "karst_slam/tests/utils/testDefines.h"
#include <mrpt/poses.h>

using namespace karst_slam;
using namespace karst_slam::gso;


int main (int argc, char* argv[])
{
    START_TEST

    std::string test_unit(argv[1]);
    if(test_unit == "check_basicGSO")
    {
        TEST((testBasicGSO<gtsam::GaussNewtonOptimizer               , GaussNewtonGSO3D>))
        TEST((testBasicGSO<gtsam::LevenbergMarquardtOptimizer        , LevenbergMarquardtGtsamGSO3D>))
        TEST((testBasicGSO<gtsam::NonlinearConjugateGradientOptimizer, NonLinearConjugateGradientGSO3D>))
        TEST((testBasicGSO<gtsam::DoglegOptimizer                    , DoglegGSO3D>))
    }
    else if(test_unit == "check_isamGSO")
    {
        TEST((testIsamGSO))
    }
    else if(test_unit == "check_isam2GSO")
    {
        TEST((testIsam2GSO))
    }
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
