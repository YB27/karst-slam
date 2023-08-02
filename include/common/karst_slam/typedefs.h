#ifndef DEF_TYPEDEFS_H
#define DEF_TYPEDEFS_H

#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>

namespace karst_slam
{
// Handy typedefs
using graph_mrpt_t  = mrpt::graphs::CNetworkOfPoses3DInf;
using pose_pdf_t   = graph_mrpt_t::constraint_t; // Pose PDF type = edge type
using pose_t       = graph_mrpt_t::constraint_no_pdf_t; // Pose type = node type
using cov_mat_t    = Eigen::Matrix<double,pose_pdf_t::state_length, pose_pdf_t::state_length>;

using points_mat_t = Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>;
using cov_t = mrpt::math::CMatrixDouble33;
template<class TYPE> using eigenAlignedVector = std::vector<TYPE, Eigen::aligned_allocator<TYPE>>;

using scanID = int;
}// end namespace

#endif // DEF_TYPEDEFS_H
