#include "karst_slam/gironaDataSetLoader/dvlLogData.h"

using namespace std;
using namespace mrpt::obs;
using namespace mrpt::poses;
using namespace karst_slam::obs;
using namespace karst_slam::UW_caveDataSet;

void dvlData::readFromLine_(stringstream &ss)
{
    string valStr;

    // Data good
    for(int i = 0; i < 4; i++)
    {
        getline(ss,valStr, ',');
        dataGood[i] = stoi(valStr);
    }

    // Altitude beam
    for(int i = 0; i < 4 ; i++)
    {
        getline(ss,valStr, ',');
        altitudeBeam [i] = stod(valStr);
    }

    // Bottom velocity beam
    for(int i = 0; i < 4 ; i++)
    {
        getline(ss,valStr, ',');
        bottomVelocityBeam [i] = stod(valStr);
    }

    // Water velocity beam
    for(int i = 0; i < 4 ; i++)
    {
        getline(ss,valStr, ',');
        waterVelocityBeam [i] = stod(valStr);
    }

    // Water velocity credit
    for(int i = 0; i < 4 ; i++)
    {
        getline(ss,valStr, ',');
        waterVelocityCredit [i] = stoi(valStr);
    }

    // Velocity Inst
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        velocityInst [i] = stod(valStr);
    }

    // Velocity Inst flag
    getline(ss,valStr, ',');
    velocityInstFlag = stoi(valStr);

    // Velocity Earth
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        velocityEarth [i] = stod(valStr);
    }

    // Velocity Earth flag
    getline(ss,valStr, ',');
    velocityEarthFlag = stoi(valStr);

    // Water Velocity Inst
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        waterVelocityInst [i] = stod(valStr);
    }

    // Water Velocity Inst flag
    getline(ss,valStr, ',');
    waterVelocityInstFlag = stoi(valStr);

    // Water Velocity Earth
    for(int i = 0; i < 3 ; i++)
    {
        getline(ss,valStr, ',');
        waterVelocityEarth[i] = stod(valStr);
    }

    // Water Velocity Earth flag
    getline(ss,valStr, ',');
    waterVelocityEarthFlag = stoi(valStr);

    // Skip roll, pitch, heading, altitude
    getline(ss,valStr, ',');
    getline(ss,valStr, ',');
    getline(ss,valStr, ',');
    getline(ss,valStr, ',');

    // Temperature
    getline(ss,valStr, ',');
    temperature = stod(valStr);

    // Pressure
    getline(ss,valStr, ',');
    pressure = stod(valStr);

    // Salinity
    getline(ss,valStr, ',');
    salinity = stod(valStr);
}

mrpt::obs::CObservationPtr dvlData::convertToCObservation()
{
    ObservationDVLPtr obs = ObservationDVL::Create();

    // ToDo : Need to better understand the meaning of all the data to only keep
    // useful information in observationDVLPtr

    obs->timestamp   = std::stoull(stamp);
    obs->sensorLabel = "DVL";
    obs->setSensorPose(CPose3D(0., 0., 0., 0., 0., 0.));

    return obs;
}

void dvlData::dumpToConsole_()
{
    cout << "++++++++ DVL ++++++++" << endl;
    cout << "Data good             : ";
    for(int i = 0; i < 4; i++)
        cout << dataGood[i] << " , ";
    cout << endl;

    cout << "Altitude beam         : ";
    for(int i = 0; i < 4; i++)
        cout << altitudeBeam[i] << " , ";
    cout << endl;

    cout << "Bottom Velocity beam  : ";
    for(int i = 0; i < 4; i++)
        cout << bottomVelocityBeam[i] << " , ";
    cout << endl;

    cout << "Water Velocity beam   : ";
    for(int i = 0; i < 4; i++)
        cout << waterVelocityBeam[i] << " , ";
    cout << endl;

    cout << "Water Velocity credit : ";
    for(int i = 0; i < 4; i++)
        cout << waterVelocityCredit[i] << " , ";
    cout << endl;

    cout << "Velocity Inst         : ";
    for(int i = 0; i < 3; i++)
        cout << velocityInst[i] << " , ";
    cout << endl;

    cout << "Velocity Inst flag      : " << velocityInstFlag << endl;

    cout << "Velocity Earth : ";
    for(int i = 0; i < 3; i++)
        cout << velocityEarth[i] << " , ";
    cout << endl;

    cout << "Velocity Earth flag     : " << velocityEarthFlag << endl;

    cout << "Water Velocity Inst : ";
    for(int i = 0; i < 3; i++)
        cout << waterVelocityInst[i] << " , ";
    cout << endl;

    cout << "Water Velocity Inst flag : " << waterVelocityInstFlag << endl;

    cout << "Water Velocity Earth : ";
    for(int i = 0; i < 3; i++)
        cout << waterVelocityEarth[i] << " , ";
    cout << endl;

    cout << "Temperature : " << temperature << endl;
    cout << "Pressure    : " << pressure    << endl;
    cout << "Salinity    : " << salinity    << endl;
}
