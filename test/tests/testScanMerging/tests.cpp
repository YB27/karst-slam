#include "testEllipticCylinder.h"

int main (int argc, char* argv[])
{
    START_TEST

    std::string test_unit(argv[1]);
    if (test_unit == "check_ellipticCylinder")
    {
        TEST((testOrthogonalDistanceToFiniteCylinder))
        TEST((testDistanceToInfiniteCylinderAlongDirection))
        TEST((testPriorRange))
        TEST((testFitting))
    }
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
