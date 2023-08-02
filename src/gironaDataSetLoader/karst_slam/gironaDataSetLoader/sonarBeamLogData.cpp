#include "karst_slam/gironaDataSetLoader/sonarBeamLogData.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

void sonar_beam::readFromLine_(stringstream& ss)
{
    string valStr;
    // Skip angle_grad
    getline(ss,valStr, ',');

    // angle (radian)
    getline(ss,valStr, ',');
    angle = stod(valStr);

    // Correspond step index
    step_idx = (int)round(angle/sonar_angular_step);

    // nbins
    getline(ss,valStr, ',');
    nbins = stoi(valStr);

    // Max range
    getline(ss,valStr, ',');
    max_range = stoi(valStr);

    // Intensities
    intensities.resize(nbins);
    for(int i = 0 ; i < nbins; i++)
    {
        getline(ss,valStr, ',');
        intensities[i] = stoi(valStr);
    }
}

CObservationPtr sonar_beam::convertToCObservation()
{
    ObservationMSISBeamPtr obs = ObservationMSISBeam::Create();

    obs->timestamp   = std::stoull(stamp);
    obs->setAngle(angle);
    obs->setIntensities(intensities);
    obs->setNAngularStep(sonar_angular_nstep);
    obs->setMaxRange(max_range);

    // In the dataset, the sonar seaking data seems wrong as there are both data for 2*Pi and 0 angles ....
    // Ignore the 2*pi data
    if(fabs(obs->getAngle() - 2.*M_PI) < 1e-4)
        return CObservationPtr();

    switch(dataType)
    {
    case SONAR_MICRON:
    {
        obs->sensorLabel = "Sonar_Micron";
        obs->setSensorPose(CPose3D(0.1, 0., -0.42, M_PI, 0., 0.)); // Pose given in the reference paper
        break;
    }
    case SONAR_SEAKING:
    {
        obs->sensorLabel = "Sonar_Seaking";
        obs->setSensorPose(CPose3D(0.55, 0., -0.15, M_PI, M_PI_2, 0.)); // Pose given in the reference paper
        break;
    }
    default:
    {
        cout << "Unknown sonar model !" << endl;
        break;
    }
    }

    return obs;
}

void sonar_beam::dumpToConsole_()
{
    cout << "++++++++ sonar_beam ++++++++" << endl;
    cout << "Angle : " << angle << endl;
    cout << "Step_idx : " << step_idx << endl;
    cout << "nbins : " << nbins << endl;
    cout << "Max range : " << max_range << endl;
    cout << "Intensities : ";
    for(const int& val : intensities)
        cout << val << " , ";
    cout << endl;
}
