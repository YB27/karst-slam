#include "karst_slam/tests/utils/testDefines.h"
#include "karst_slam/simulation/observationSimulator_sonar.h"

int testRangeQuantification()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::sensors;
    using namespace karst_slam::simulation;

    std::shared_ptr<igl::embree::EmbreeIntersector> dummy_embreeIntersector;
    observationSimulator_sonar obsSimulator(dummy_embreeIntersector,
                                            "dummy_observator");
    sonarCharacteristics sc;
    sc.quantification_range= 0.2;

    obsSimulator.setSonarCharacteristics(sc);

    vector<double> ranges = {1., 1.1, 0.76, 2.45, 3.77, 4.32, 8.21};
    int n = ranges.size();

    vector<double> expectedQuantification = {1., 1., 0.8, 2.4, 3.8, 4.4, 8.2};
    vector<double> computedQuantification;
    computedQuantification.reserve(n);
    for(const double r : ranges)
        computedQuantification.push_back(obsSimulator.rangeQuantification(r));

    bool hasFailed = false;
    for(int i = 0; i < n; i++)
    {
        if(fabs(expectedQuantification[i] - computedQuantification[i]) > 1e-8)
        {
            hasFailed = true;
            break;
        }
    }

    if(hasFailed)
    {
        FAILED
        cout << "Expected quantification: " << endl;
        for(const double r : expectedQuantification)
            cout << r << " , ";
        cout << endl;
        cout << "Computated quantification: " << endl;
        for(const double r: computedQuantification)
            cout << r << " , ";
        cout << endl;
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}

int testFilterRanges()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::sensors;
    using namespace karst_slam::simulation;

    std::shared_ptr<igl::embree::EmbreeIntersector> dummy_embreeIntersector;
    observationSimulator_sonar obsSimulator(dummy_embreeIntersector,
                                            "dummy_observator");
    sonarCharacteristics sc;
    sc.filterRanges = true;
    sc.filterRanges_minDelta = 0.5;

    obsSimulator.setSonarCharacteristics(sc);

    vector<double> ranges = {1., 1.2, 1.21 ,2.05, 2.5, 3.2, 5.4, 5.8, 6.2, 7.};
    vector<sonarMeasure> rawMeasures;
    rawMeasures.reserve(ranges.size());
    for(const double& r : ranges)
        rawMeasures.push_back(sonarMeasure(r,0.,0.));

    vector<sonarMeasure> filteredMeasures = obsSimulator.filterRanges(rawMeasures);

    vector<sonarMeasure> expectedFilteredMeasures = {sonarMeasure(1.   , 0., 0.),
                                                     sonarMeasure(2.05 , 0., 0.),
                                                     sonarMeasure(3.2  , 0., 0.),
                                                     sonarMeasure(5.4  , 0., 0.),
                                                     sonarMeasure(6.2  , 0., 0.),
                                                     sonarMeasure(7.   , 0., 0.)};

    bool hasFailed = false;
    int n = filteredMeasures.size();
    if(n != expectedFilteredMeasures.size())
        hasFailed = true;
    else
    {
        for(int i = 0; i < n; i++)
        {
            if(filteredMeasures[i].range != expectedFilteredMeasures[i].range)
            {
                hasFailed = true;
                break;
            }
        }
    }

    if(hasFailed)
    {
        FAILED
        cout << "Expected filtered measures : " << endl;
        for(const sonarMeasure& sm : expectedFilteredMeasures)
            cout << sm.range << " , ";
        cout << endl;
        cout << "Computated filtered measures : " << endl;
        for(const sonarMeasure& sm: filteredMeasures)
            cout << sm.range << " , ";
        cout << endl;
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}