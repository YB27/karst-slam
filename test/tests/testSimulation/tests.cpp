#include "testObservationSimulatorSonar.h"

int main (int argc, char* argv[])
{
    START_TEST

    std::string test_unit(argv[1]);
    if(test_unit == "check_observationSimulatorSonar")
    {
        TEST((testRangeQuantification))
        TEST((testFilterRanges))
    }
    else
        std::cout << "Unknown test unit name !" << std::endl;

    END_TEST
}
