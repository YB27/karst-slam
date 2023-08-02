#include "karst_slam/saveToFiles.h"
#include <mrpt/poses/CPose3DPDFGaussian.h>

using namespace karst_slam::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace std;

void writeCovarianceToFile(const CMatrixDouble66& cov,
                           ofstream& file)
{
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
            file << "," << cov(i, j);
}

template<>
void karst_slam::utils::writePoseToFile<CPose3D>(const CPose3D& pose,
                                                 ofstream& file)
{
    for(int i = 0; i < 3 ; i++)
        file << pose.m_coords[i] << ",";
    file << pose.yaw()   << ","
         << pose.pitch() << ","
         << pose.roll();
}

template<>
void karst_slam::utils::writePoseToFile<CPose3DPDFGaussian>(const CPose3DPDFGaussian& pose,
                                                            ofstream& file)
{
    writePoseToFile(pose.mean, file);
    writeCovarianceToFile(pose.cov, file);
}