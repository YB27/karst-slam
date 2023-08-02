#include "testMSIS.h"

int main (int argc, char* argv[])
{
    START_TEST

    std::string test_unit(argv[1]);
//    if(test_unit == "check_polarToCart")
//        TEST((testMSIS_polarToCartesianCovariance))
    if (test_unit == "check_intensitiesToCart")
        TEST((testMSIS_intensitiesToCartesianCoords))
    else if(test_unit == "check_generateFullScan")
        TEST((testMSIS_generateFullScan))
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
