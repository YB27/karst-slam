#include "karst_slam/gui/utils.h"
#include <Eigen/Core>
#include <vector>

using points_mat_t = Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::RowMajor>;

using namespace std;

// Convertion needed for point cloud display
void karst_slam::utils::pointsMatrixToCoordVectors(const points_mat_t &pointsGlobalPoses,
                                                   int nPoints,
                                                   vector<float>& x_vals, vector<float>& y_vals, vector<float>& z_vals)
{
    // Cast the points matrix to float (CPointCloud only accept float points)
    Eigen::Matrix<float,4,Eigen::Dynamic, Eigen::RowMajor> points_float = pointsGlobalPoses.cast<float>();

    const float* x_row_begin = points_float.data();
    const float* x_row_end   = x_row_begin + nPoints;
    x_vals = vector<float>(x_row_begin, x_row_end);
    const float* y_row_end = x_row_end + nPoints;
    y_vals = vector<float>(x_row_end, y_row_end);
    const float* z_row_end = y_row_end + nPoints;
    z_vals = vector<float>(y_row_end, z_row_end);
}

