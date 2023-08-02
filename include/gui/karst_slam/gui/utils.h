#ifndef GUI_UTILS_H
#define GUI_UTILS_H

#include <vector>
#include <Eigen/Core>

namespace karst_slam{namespace utils{

/** Converts a matrix of points to three vectors of each coordinates */
void pointsMatrixToCoordVectors(const Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor> &pointsGlobalPoses,
                                       int nPoints,
                                       std::vector<float>& x_vals, std::vector<float>& y_vals, std::vector<float>& z_vals);

}}

#endif // GUI_UTILS_H
