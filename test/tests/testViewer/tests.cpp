#include "testGraphSlamEngineViewer.h"
#include "testUtils.h"
#include "karst_slam/tests/utils/testDefines.h"
#include <mrpt/poses.h>

int main (int argc, char* argv[])
{
    START_TEST

    std::string test_unit(argv[1]);
    if(test_unit == "check_basicDisplay")
    {
        TEST((testDisplay))
    }
    else if(test_unit == "check_EigenMatToVecConversion")
    {
        TEST((test_pointsMatrixToCoordVectors))
    }
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
