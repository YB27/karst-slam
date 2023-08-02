#include "karst_slam/sensors/gui/MSISViewer.h"
#include "karst_slam/sensors/Scan.h"
#include "karst_slam/gui/utils.h"

using namespace std;

int test_pointsMatrixToCoordVectors()
{
    int nPoints = 6;
    points_mat_t pointsMat(4,nPoints);
    pointsMat << 1., 2., 3., 4., 5., 6.,
                 7., 8.6, 9.785, 1. ,2.,5.,
                 -1.2, 1.02365, -0.589, 0.6, 4.2, 4.8,
                 0., 5.3, 1.4, 1.2, 7.6, 1.36;

    vector<float> x_vals, y_vals, z_vals;
    karst_slam::utils::pointsMatrixToCoordVectors(pointsMat, nPoints, x_vals, y_vals, z_vals);

    bool hasFail = false;
    for(int i = 0; i < nPoints; i++)
    {
        hasFail = (fabs(x_vals[i] - pointsMat(0,i)) > 1e-5) ||
                  (fabs(y_vals[i] - pointsMat(1,i)) > 1e-5) ||
                  (fabs(z_vals[i] - pointsMat(2,i)) > 1e-5);
        if(hasFail)
            break;
    }

    if(hasFail)
    {
        cout << "[FAILED]  " << __PRETTY_FUNCTION__ << endl;
        cout << "Original matrix points : " << endl;
        cout << pointsMat << endl;
        cout << "Computed vectors x,y and z: " << endl;
        for(const float& v : x_vals)
            cout << v << ",";
        cout << endl;
        for(const float& v : y_vals)
            cout << v << ",";
        cout << endl;
        for(const float& v : z_vals)
            cout << v << ",";
        return -1;
    }
    else
    {
        cout << "[PASSED]  " << __PRETTY_FUNCTION__ << endl;
        return 0;
    }

}
