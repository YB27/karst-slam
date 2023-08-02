#include "karst_slam/scanMerging/surfaceTrainingData.h"
#include "karst_slam/scanMerging/rangeMapping.h"
#include <fstream>
#include <cmath>

using namespace std;
using namespace karst_slam::scanMerging;
using namespace mrpt::poses;

surfaceTrainingData::surfaceTrainingData() : surfaceData<surfaceDatum>()
{
    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

void surfaceTrainingData::computeTrainingDataWithPrior(const ellipticCylinder& priorCylinder,
                                                       const map<unsigned long long, trajectoryPose<CPose3DPDFGaussian>>& poses,
                                                       const CPose3D& verticalSonarPoseOnRobot)
{
    double r_prior = 0.;
    CPose3D pose_sonar;
    for(surfaceDatum& datum : m_data)
    {
        // ToDo : precompute cos(yaw), sin(yaw) values somewhere
        // ToDo2 : Also precompute all the sonar pose
        r_prior = rangeMapping::rangeMap(surfaceValidationData::priorRange(priorCylinder, poses.at(datum.timeStamp).pose.mean + verticalSonarPoseOnRobot, datum.yaw));
        datum.r_prior = r_prior;
        if(r_prior > 0)
            datum.out_r -= r_prior;
        else
            MRPT_LOG_ERROR_STREAM("[surfaceTrainingData::computeTrainingDataWithPrior] No range for training data !! ");
    }
}

void surfaceTrainingData::save(const string& fileName) const
{
    ofstream f(fileName);
    if(f.is_open())
    {
        f << "#s,yaw,out_r, r_prior, isCensored" << endl;
        for(const surfaceDatum& datum : m_data)
        {
            f << datum.s << ","
              << datum.yaw << ","
              << datum.out_r << ","
              << datum.r_prior << ","
              << (datum.isCensored ? 1 : 0)  
              << "\n";
        }
        f.close();
    }
    else
        MRPT_LOG_ERROR_STREAM("Can't open file " + fileName);    
}

void surfaceTrainingData::saveAbscissa(const string& fileName_abs) const 
{
// TMP save abscissa in separate file as before 
    ofstream fileAbs(fileName_abs);
    if(fileAbs.is_open())
    {
        fileAbs << "#timeStamp, abscissa" << endl;
        for(const surfaceDatum& datum : m_data)
        {
            fileAbs << datum.timeStamp << "," << datum.s << endl;
        }
        fileAbs.close();
    }
    else
        MRPT_LOG_ERROR_STREAM("Can't open file " + fileName_abs);
}

void surfaceTrainingData::load(const string &fileName)
{
    m_data.clear();

    ifstream file(fileName);
    string line, val;
    surfaceDatum datum;
    if(file.is_open())
    {
        getline(file,line); // skip header
        while(getline(file,line))
        {
             stringstream ss(line);

             getline(ss,val,',');
             datum.s = stod(val);

             getline(ss,val,',');
             datum.yaw = stod(val);

             getline(ss,val,',');
             datum.out_r = stod(val);

             getline(ss,val,',');
             datum.r_prior = stod(val);

             m_data.push_back(datum);
        }
        file.close();
    }
    else
        MRPT_LOG_ERROR_STREAM("Can't open file " << fileName);
}

void surfaceTrainingData::append(const surfaceTrainingData& otherData)
{
    if(size() == 0)
        *this = otherData;
    else
    {    
        m_data.reserve(size() + otherData.size());
        for(const surfaceDatum& d : otherData.m_data)
        {
            surfaceDatum dat  = surfaceDatum(d.s,
                                             d.yaw,
                                             d.out_r);
            dat.timeStamp = d.timeStamp;
            dat.pt        = d.pt;
            dat.r_prior   = d.r_prior;
            dat.isCensored = d.isCensored;
            m_data.push_back(move(dat));
        }
    }
}

void surfaceTrainingData::computeAbscissa(const ellipticCylinder& priorCylinder,
                                          double& min_abscissa,
                                          double& max_abscissa)
{
    min_abscissa = numeric_limits<double>::max();
    max_abscissa = numeric_limits<double>::min();
    for(surfaceDatum& sd : m_data)
    {
        // Get sd 
        surfaceValidationData::polarCoordsProjOnGuide(priorCylinder, m_isLearningMeanFunc, sd);

        // Update min/max abscissa
        if(sd.s > max_abscissa)
            max_abscissa = sd.s;
        else if( sd.s < min_abscissa)
            min_abscissa = sd.s;
    }
}