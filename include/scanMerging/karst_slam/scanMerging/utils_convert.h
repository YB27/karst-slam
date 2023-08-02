#ifndef DEF_UTILS_CONVERT_H
#define DEF_UTILS_CONVERT_H

#include <opencv/cv.h>
#include <mrpt/poses/CPointPDFGaussian.h>

namespace karst_slam{namespace utils
{
    /**
     * Handy function to convert a vector of N 3D points to a Nx3 Matrix
     * 
     * @param points Vector of points
     * @return Nx3 Matrix
     */
    static cv::Mat convertTo_cvMat(const std::vector<mrpt::poses::CPointPDFGaussian>& points)
    {
        int N = points.size();
        cv::Mat data_pts(N,3,CV_64F);
        double* dataPtr = data_pts.ptr<double>(0);
        for(const mrpt::poses::CPointPDFGaussian& pt : points)
        {
            const mrpt::poses::CPoint3D& pt_mean = pt.mean;
            *(dataPtr++) = pt_mean.x();
            *(dataPtr++) = pt_mean.y();
            *(dataPtr++) = pt_mean.z();
        }
        return data_pts;
    }
}}
#endif // DEF_UTILS_CONVERT_H
