#include "test_poseGraph.h"
int main(int argc, char* argv[])
{
    START_TEST
    std::string test_unit(argv[1]);
    if(test_unit == "check_relativePosePdf")
    {
        TEST((testRelativePosePdf))
    }
    else if(test_unit == "check_marginals")
    {
        TEST((testMarginals))
    }
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
