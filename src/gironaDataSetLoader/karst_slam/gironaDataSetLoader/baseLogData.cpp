#include "karst_slam/gironaDataSetLoader/baseLogData.h"

using namespace std;
using namespace karst_slam::UW_caveDataSet;

void data::readFromLine(stringstream& ss,
                        DATA_FROM_SENSOR d)
{
    dataType = d;

    string valStr;
    // Skip time and seq idx
    getline(ss,valStr, ',');
    getline(ss,valStr, ',');

    // Stamp
    getline(ss,valStr, ',');
    stamp = valStr;

    readFromLine_(ss);
}

void data::dumpToConsole()
{
    cout << "Stamp : " << stamp << endl;
    dumpToConsole_();
}
