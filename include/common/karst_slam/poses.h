#ifndef KARST_SLAM_POSES_H
#define KARST_SLAM_POSES_H

#include "mrpt_poses_includes.h"

namespace karst_slam{

/** Convert a 3D pose gaussian pdf to a 3D pose gaussian inf pdf (ie using information matrix instead of covariance matrix) */
static mrpt::poses::CPose3DPDFGaussianInf convertToGaussianInf(const mrpt::poses::CPose3DPDFGaussian& pose)
{
    mrpt::math::CMatrixDouble66 inf_cov;
    pose.getInformationMatrix(inf_cov);
    return mrpt::poses::CPose3DPDFGaussianInf(pose.getMeanVal(), inf_cov);
}

/** Convert a 3D pose gaussian inf pdf (ie using information matrix instead of covariance matrix) to a 3D pose gaussian pdf  */
static mrpt::poses::CPose3DPDFGaussian convertToGaussian(const mrpt::poses::CPose3DPDFGaussianInf& pose)
{
    mrpt::math::CMatrixDouble66 cov;
    pose.getCovariance(cov);
    return mrpt::poses::CPose3DPDFGaussian(pose.getMeanVal(), cov);
}

/** Inverse a pose pdf */
template<class POSE_PDF_T>
static POSE_PDF_T inversePosePdf(const POSE_PDF_T& pose)
{
    POSE_PDF_T pose_inverse;
    pose.inverse(pose_inverse);
    return pose_inverse;
}

/** Extract the vector of pose means from a vector of pose pdf*/
template<class POSE_PDF_T>
static std::vector<mrpt::poses::CPose3D> getVectorOfMeans(const std::vector<POSE_PDF_T>& vec_pdf)
{
    std::vector<mrpt::poses::CPose3D> vec_mean;
    vec_mean.reserve(vec_pdf.size());
    for (const POSE_PDF_T& posePDF : vec_pdf)
        vec_mean.push_back(posePDF.mean);

    return vec_mean; 
}

} // end namespaces
#endif
