#include "karst_slam/tests/utils/testDefines.h"
#include "karst_slam/scanMerging/ellipticCylinder.h"
#include "karst_slam/scanMerging//surfaceValidationData.h"

std::vector<mrpt::poses::CPointPDFGaussian> generateSampleDataOnCylinder_sonarLike(const karst_slam::scanMerging::ellipticCylinder& cylinder,
                                                                                   double dx,
                                                                                   double dangle)
{
    using namespace std;
    using namespace mrpt::poses;

    vector<CPointPDFGaussian> pts;

    const CPose3D& pose = cylinder.getPose();
    double a = cylinder.getHalfLongAxis(), b = cylinder.getHalfSmallAxis(), length = cylinder.getLength();
    double angle = 0.;
    int N = length/(double)dx;
    double x = -0.5*length;
    for(int i = 0; i <= N; i++)
    {
        CPoint3D localPt(x, a*cos(angle), b*sin(angle));

        CPointPDFGaussian pt;
        mrpt::math::TPoint3D globalPt;
        pose.composePoint(localPt,globalPt);
        pt.mean = CPoint3D(globalPt);

        pts.push_back(pt);

        angle += dangle;
        x += dx;
    }

    return pts;
}

int testOrthogonalDistanceToFiniteCylinder()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::scanMerging;

    ellipticCylinder cylinder(CPose3D(0,0,0,0,0,0), //CPose3D(1,-1,2,M_PI/4.,-M_PI/4.,M_PI/4.), // Pose
                                 1., // Half long axis
                                 0.5, // Half small axis
                                 4.); // length

    vector<double> expectedValues, computatedValues;

    CPoint3D p(0,0,0);
    expectedValues.push_back(0.5);
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(-1.5,5,0);
    expectedValues.push_back(4.);
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(1.2,0,-4);
    expectedValues.push_back(3.5);
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(3.,0.5,0.25);
    expectedValues.push_back(1.);
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(-3.,-0.5,-0.25);
    expectedValues.push_back(1.);
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(4.,2,0.);
    expectedValues.push_back(sqrt(5));
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(-4.,2,0.);
    expectedValues.push_back(sqrt(5));
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    p = CPoint3D(4.,0.,1.5);
    expectedValues.push_back(sqrt(5));
    computatedValues.push_back(cylinder.getOrthogonalDistanceToFiniteCylinder(p));

    bool hasFailed = false;
    for(int i = 0; i < expectedValues.size(); i++)
    {
        if(fabs(expectedValues[i] - computatedValues[i]) > 1e-4)
        {
            hasFailed = true;
            break;
        }
    }

    if(hasFailed)
    {
        FAILED
        cout << "Expected distances : " << endl;
        for(double d : expectedValues)
            cout << d << " , ";
        cout << endl;
        cout << "Computated distances : " << endl;
        for(double d : computatedValues)
            cout << d << " , ";
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}

int testDistanceToInfiniteCylinderAlongDirection()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::scanMerging;

    ellipticCylinder cylinder(CPose3D(0,0,0,0,0),
                              3.,
                              0.5,
                              2);

    // Direction is orthogonal to cylinder axis
    Eigen::Vector3d pt, dir;
    vector<double> expected_distances, computated_distances;

    // y - long axis
    dir << 0,1,0;

    pt  << 0,0,0;
    expected_distances.push_back(3.);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    pt << -1,0,0;
    expected_distances.push_back(3.);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    pt << -1,1,0;
    expected_distances.push_back(2.);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // z - small axis
    dir << 0,0,1;

    pt << 0,0,0;
    expected_distances.push_back(0.5);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    pt << -1,0,0;
    expected_distances.push_back(0.5);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    pt << -1,0,0.25;
    expected_distances.push_back(0.25);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // x - cylinder axis
    dir << 1,0,0;

    // infinite distance
    pt << 0,0,0;
    expected_distances.push_back(-1);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // Other directions
    double angle = M_PI/6., cs = cos(angle), ss = sin(angle);

    // In XY plane
    dir << cs, ss, 0.;
    pt << 0,0,0;
    expected_distances.push_back(3./ss);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // IN XZ plane
    dir << cs, 0., ss;
    expected_distances.push_back(0.5/ss);
    computated_distances.push_back(cylinder.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // Test with a rotated cylinder
    ellipticCylinder cylinder_rotated(CPose3D(0,0,0,angle,0,0),
                              3.,
                              0.5,
                              2);
    pt << 0,0,0;
    dir << 0,1,0;
    expected_distances.push_back(3./cs);
    computated_distances.push_back(cylinder_rotated.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    // Test with translated cylinder
    ellipticCylinder cylinder_translated(CPose3D(1,-1,0,0,0,0),
                                         3,
                                         0.5,
                                         2);
    pt << 2, 0, 0;
    dir << 0,1,0;
    expected_distances.push_back(2);
    computated_distances.push_back(cylinder_translated.getDistanceToInfiniteCylinderAlongDirection(pt, dir));

    bool hasFailed = false;
    for(int i = 0; i < expected_distances.size(); i++)
    {
        if(fabs(expected_distances[i] - computated_distances[i]) > 1e-4)
        {
            hasFailed = true;
            break;
        }
    }

    if(hasFailed)
    {
        FAILED
        cout << "Expected distances : " << endl;
        for(double d : expected_distances)
            cout << d << " , ";
        cout << endl;
        cout << "Computated distances : " << endl;
        for(double d : computated_distances)
            cout << d << " , ";
        cout << endl;
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}

int testPriorRange()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::scanMerging;

    ellipticCylinder cylinder(CPose3D(0,0,0,0,0,0),
                              3.,
                              0.5,
                              2);

    CPose3D pose;
    double yaw;
    vector<double> expected_distances, computated_distances;

    // Observation in XY plane of the cylinder
    pose = CPose3D(0.,0.,0.,0.,0.,0.);
    yaw = 0.;
    expected_distances.push_back(-1);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    yaw = M_PI/2.;
    expected_distances.push_back(3);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    pose = CPose3D(0.1,1.,0.,0.,0.,0.);
    yaw = 0.;
    expected_distances.push_back(-1);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    yaw = M_PI/2.;
    expected_distances.push_back(2);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    // Observation in the XZ plane of the cylinder
    pose = CPose3D(0.,0.,0.,0.,0.,M_PI/2.);
    yaw = 0.;
    expected_distances.push_back(-1);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    // No intersection with the cylinder -> should return -1
    yaw = M_PI/2.;
    expected_distances.push_back(0.5);
    computated_distances.push_back(surfaceValidationData::priorRange(cylinder, pose, yaw)/*cylinder.priorRange(pose, yaw)*/);

    bool hasFailed = false;
    for(int i = 0; i < expected_distances.size(); i++)
    {
        if(fabs(expected_distances[i] - computated_distances[i]) > 1e-4)
        {
            hasFailed = true;
            break;
        }
    }

    if(hasFailed)
    {
        FAILED
        cout << "Expected distances : " << endl;
        for(double d : expected_distances)
            cout << d << " , ";
        cout << endl;
        cout << "Computated distances : " << endl;
        for(double d : computated_distances)
            cout << d << " , ";
        cout << endl;
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }
}

int testFitting()
{
    using namespace std;
    using namespace mrpt::poses;
    using namespace karst_slam::scanMerging;

    ellipticCylinder cylinder_gt(CPose3D(1,-1,2,0,0,0), //CPose3D(1,-1,2,M_PI/4.,-M_PI/4.,M_PI/4.), // Pose
                                 1., // Half long axis
                                 0.5, // Half small axis
                                 4.), // length
                     cylinder_fit;

    // Generate synthetic data
    vector<CPointPDFGaussian> testPts = generateSampleDataOnCylinder_sonarLike(cylinder_gt,
                                                                               0.1,
                                                                               2.*M_PI/20.);

    cylinder_fit.fit(testPts, CPose3D(), vector<CPose3DPDFGaussian>());

    cout << "Expected Cylinder : " << endl;
    cylinder_gt.print();
    cout << "Computated Cylinder : " << endl;
    cylinder_fit.print();

    if(cylinder_gt.equalTo(cylinder_fit, 1e-3))
    {
        FAILED
        cout << "Expected Cylinder : " << endl;
        cylinder_gt.print();
        cout << "Computated Cylinder : " << endl;
        cylinder_fit.print();
        return -1;
    }
    else
    {
        PASSED
        return 0;
    }

}
