#include "karst_slam/simulation/simulationTrajectory_customFile.h"
#include <mrpt/random.h>

using namespace std;
using namespace karst_slam::simulation;
using namespace karst_slam::scanMerging;
using namespace mrpt::utils;
using namespace mrpt::poses;


simulationTrajectory_customFile::simulationTrajectory_customFile(const CConfigFile &cfg) : simulationTrajectory_base(cfg)
{
    m_type = CUSTOM_FILE;
    if (m_useAbsoluteDepthPitchRoll)
    {
        string odometryAbsoluteFile = cfg.read_string("Simulation", "odometryAbsoluteFile", "", true);
        loadAbsoluteDepthPitchRoll(odometryAbsoluteFile);
    }

    string trajectoryFileName = cfg.read_string("Simulation", "trajectoryFile", "", true);
    loadFrom(trajectoryFileName);
}

void simulationTrajectory_customFile::loadAbsoluteDepthPitchRoll(const string& fileName)
{
    ifstream file(fileName);
    if (file.is_open())
    {
        string line, val;

        getline(file, line); // skip header
        while (getline(file, line))
        {
            stringstream ss(line);
            getline(ss, val, ','); // x
            getline(ss, val, ','); // y
            getline(ss, val, ','); // z
            m_absoluteZ.push_back(stod(val));
            getline(ss, val, ','); // yaw 
            getline(ss, val, ','); // pitch
            m_absolutePitch.push_back(stod(val));
            getline(ss, val, ','); // roll
            m_absoluteRoll.push_back(stod(val));
        }

        file.close();
    }
    else
       MRPT_LOG_ERROR_STREAM("Can't open file " << fileName);
    
    MRPT_LOG_DEBUG_STREAM("m_absoluteZ size : " << m_absoluteZ.size());
}

void simulationTrajectory_customFile::loadFrom(const string &fileName)
{
    double pose[6];
    CPose3DPDFGaussian incr;
    ifstream file(fileName);
    if (file.is_open())
    {
        string line, val;

        getline(file, line); // skip header
        while (getline(file, line))
        {
            stringstream ss(line);

            // Mean
            for (int i = 0; i < 6; i++)
            {
                getline(ss, val, ',');
                pose[i] = stod(val);
            }

            incr.mean = CPose3D(pose[0], pose[1], pose[2],
                                pose[3], pose[4], pose[5]);

            // Covariance matrix (supposed diagonal)
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    getline(ss, val, ',');
                    incr.cov(i, j) = stod(val);
                }
            }
            m_customLocalIncrements.push_back(incr);
        }

        file.close();

        if (m_type == CUSTOM_FILE)
        {
            m_drPose_GT = m_customLocalIncrements;
            if (m_noisyTraj)
            {
                for (const CPose3DPDFGaussian& incr : m_customLocalIncrements)
                {
                    CPose3DPDFGaussian sampleIncr(incr);
                    incr.drawSingleSample(sampleIncr.mean);
                    m_drPose.push_back(sampleIncr);
                }

                for (int i = 0; i < m_absoluteZ.size(); i++)
                {
                    m_absoluteZ_samples.push_back(mrpt::random::randomGenerator.drawGaussian1D(m_absoluteZ[i], sqrt(m_absoluteZ_var)));
                    m_absolutePitch_samples.push_back(mrpt::random::randomGenerator.drawGaussian1D(m_absolutePitch[i], sqrt(m_absoluteAngle_var)));
                    m_absoluteRoll_samples.push_back(mrpt::random::randomGenerator.drawGaussian1D(m_absoluteRoll[i], sqrt(m_absoluteAngle_var)));
                }
                MRPT_LOG_DEBUG_STREAM("m_absoluteZ_samples size      : " << m_absoluteZ_samples.size());
                MRPT_LOG_DEBUG_STREAM("m_absolutePitch_samples size  : " << m_absolutePitch_samples.size());
                MRPT_LOG_DEBUG_STREAM("m_absoluteRoll_samples size   : " << m_absoluteRoll_samples.size());
            }
            else
                m_drPose= m_drPose_GT;  
        }
    }
    else
        MRPT_LOG_ERROR_STREAM("Can't open file " << fileName);
}

trajectoryPose<CPose3DPDFGaussian> simulationTrajectory_customFile::getPose(bool /*updateOdo*/,
                                                                            uint64_t timeStamp,
                                                                            const trajectoryPose<CPose3DPDFGaussian>& trajPrevPose)
{
    trajectoryPose<CPose3DPDFGaussian> trajPose;
    trajPose.timeStamp = timeStamp;

    trajPose.pose_gt = trajPrevPose.pose_gt + m_drPose_GT[m_lastIdx];
    trajPose.pose = trajPrevPose.pose + m_drPose[m_lastIdx];
    if (m_useAbsoluteDepthPitchRoll)
    {
        // Fix the z, pitch and roll values
        trajPose.pose.mean.setFromValues(trajPose.pose.mean.x(), trajPose.pose.mean.y(), m_absoluteZ_samples[timeStamp],
                                         trajPose.pose.mean.yaw(), m_absolutePitch_samples[timeStamp], m_absoluteRoll_samples[timeStamp]);

        fixDepthPitchRoll(trajPose);
    }

    m_lastIdx++;
    return trajPose; 
}