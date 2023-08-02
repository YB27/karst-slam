#include "karst_slam/scanMerging/ellipticCylinder.h"
#include "karst_slam/scanMerging/LM.h"
#include "karst_slam/compare.h"
#include "karst_slam/scanMerging/utils_convert.h"
#include <mrpt/math/poly_roots.h>
#include <opencv2/imgproc.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <include/grassmann_pca.hpp>
#include <include/private/boost_ublas_row_iterator.hpp>

using namespace std;
using namespace karst_slam::scanMerging;
using namespace mrpt::poses;
using namespace mrpt::math;

const double txy = sqrt(2.)/2.;// Value used to compute distance to ellipse

ellipticCylinder::ellipticCylinder(const CPose3D &pose, double halfLongAxis, double halfSmallAxis, double length):
    m_halfLongAxis(halfLongAxis),
    m_halfSmallAxis(halfSmallAxis),
    m_length(length)
{
    m_principalBase       = Eigen::Matrix3d::Zero();
    m_principalBase_t     = Eigen::Matrix3d::Zero();
    m_principalBase_pca   = Eigen::Matrix3d::Zero();
    m_principalBase_t_pca = Eigen::Matrix3d::Zero();
    m_pose_pca = CPose3D();
    m_halfLongAxis_pca = 0.;
    m_halfSmallAxis_pca = 0.;
    m_length_pca = 0.;
    setPose(pose);

    this->setMinLoggingLevel(mrpt::utils::LVL_DEBUG);
}

void ellipticCylinder::fit(const vector<CPointPDFGaussian>& points,
                           const CPose3D& verticalSonarPoseOnRobot,
                           const vector<CPose3DPDFGaussian>& bodyPoses)
{
    // Initial cylinder
    fitPCA(points);
    MRPT_LOG_INFO_STREAM("[ellipticCylinder::fit] Cylinder after pca : ");
    print();

    // Refinement with non linear least square
    auxData_fitNLLS auxData(points, verticalSonarPoseOnRobot, bodyPoses, getPose());
    fitNLLS_infinite(auxData);

    MRPT_LOG_INFO_STREAM("[ellipticCylinder::fit] Cylinder after NLLS : ");
    print();
}

void ellipticCylinder::fit(const vector<CPointPDFGaussian>& points,
                           const CPose3D& verticalSonarPoseOnRobot,
                           const vector<CPose3D>& bodyPoses)
{
    // Initial cylinder
    fitPCA(points);
    print();

    const CPose3D& cylinderPose_pca = getPose();
    std::cout << "cylinderPose_pca :" << std::setprecision(std::numeric_limits<long double>::digits10 + 1) 
                                      << cylinderPose_pca.m_coords[0] << "," 
                                      << cylinderPose_pca.m_coords[1] << "," 
                                      << cylinderPose_pca.m_coords[2] << "," 
                                      << cylinderPose_pca.yaw() << "," 
                                      << cylinderPose_pca.pitch() << ","
                                      << cylinderPose_pca.roll() << ","
                                      << std::endl;  

    // Refinement with non linear least square
    auxData_fitNLLS auxData(points, verticalSonarPoseOnRobot, bodyPoses, getPose());
    fitNLLS_infinite(auxData);

    MRPT_LOG_INFO_STREAM("[ellipticCylinder::fit] Cylinder after NLLS : ");
    print();
}

void ellipticCylinder::fitNLLS_finite(const auxData_fitNLLS& auxData)
{
    // Convert the ellipticCylinder in the values to optimize
    // Here it is a 9D vector : 6D for se(3), 2 for ellipse axis radii and 1 for length
    mrpt::math::CVectorDouble init_vals(9);
    for(int i = 0; i < 6; i++)
        init_vals[i] = 0.;
    init_vals[6] = m_halfLongAxis;
    init_vals[7] = m_halfSmallAxis;
    init_vals[8] = m_length;

    CVectorDouble increments(9);
    increments.fill(0.001);

    mrpt::math::CVectorDouble opt_vals;
    mrpt::math::LM_modifyTempl<mrpt::math::CVectorDouble, auxData_fitNLLS> lm;

    CVectorDouble initError(1);
    errorFunctionForLM_finite(init_vals,auxData,initError);

    mrpt::math::LM_modifyTempl<mrpt::math::CVectorDouble, auxData_fitNLLS>::TResultInfo info;
    lm.execute(opt_vals,                                       /* out_optimal_x*/
               init_vals,                                      /* x0 */ 
               &ellipticCylinder::errorFunctionForLM_finite,   /* functor */
               increments,                                     /* increments */
               auxData,                                        /* userParam */
               info,                                           /* out_info */
               mrpt::utils::LVL_ERROR,                         /* verbosity */
               100,                                            /* maxIter */ 
               1e-3,                                           /* tau */
               1e-8,                                           /* e1 */
               0,                                              /* e2 */
               true,                                           /* returnPath */
               &ellipticCylinder::incrementAdder_LM_finite);   /* x_increment_adder */

    // Set to estimated cylinder
    setPose(auxData.curEstimatedPose);
    m_halfLongAxis  = opt_vals[6];
    m_halfSmallAxis = opt_vals[7];
    m_length        = opt_vals[8];
}

void ellipticCylinder::fitNLLS_infinite(const auxData_fitNLLS& auxData)
{
    // Convert the ellipticCylinder in the values to optimize
    // Here it is a 8D vector : 6D for se(3), 2 for ellipse axis radii
    mrpt::math::CVectorDouble init_vals(8);
    for(int i = 0; i < 6; i++)
        init_vals[i] = 0.;
    init_vals[6] = m_halfLongAxis;
    init_vals[7] = m_halfSmallAxis;

    CVectorDouble increments(8);
    increments.fill(0.01);

    mrpt::math::CVectorDouble opt_vals;
    mrpt::math::LM_modifyTempl<mrpt::math::CVectorDouble, auxData_fitNLLS> lm;

    CVectorDouble initError(1);
    // Use a regularisation on the distance from points to the cylinder axes
    errorFunctionForLM_infinite_constrained(init_vals,auxData,initError); 
    //errorFunctionForLM_infinite(init_vals,auxData,initError);

    mrpt::math::LM_modifyTempl<mrpt::math::CVectorDouble, auxData_fitNLLS>::TResultInfo info;
    lm.execute(opt_vals,                                                    /* out_optimal_x*/
               init_vals,                                                   /* x0 */
               //&ellipticCylinder::errorFunctionForLM_infinite,
               &ellipticCylinder::errorFunctionForLM_infinite_constrained,  /* functor */
               increments,                                                  /* increments */
               auxData,                                                     /* userParam */
               info,                                                        /* out_info */
               mrpt::utils::LVL_ERROR,                                      /* verbosity */
               100,                                                         /* maxIter */
               1e-3,                                                        /* tau */
               1e-8,                                                        /* e1 */
               0,                                                           /* e2 */
               true,                                                        /* returnPath */
               &ellipticCylinder::incrementAdder_LM_infinite);              /* x_increment_adder */

    // Set to estimated cylinder
    setPose(auxData.curEstimatedPose);
    m_halfLongAxis  = opt_vals[6];
    m_halfSmallAxis = opt_vals[7];
}

void ellipticCylinder::setAxisFromTangentVectorsProjections(const Eigen::Vector3d &dotProds)
{
    // If the height is the superior to the major axis of ellipses, then it is simply the first eigenvector
    // However, in the general case, need to find this axis
    // Here the idea is that the dot product of tangents with the main axis should be near zero
    if(dotProds(1) < dotProds(0) && dotProds(1) < dotProds(2))
    {
        MRPT_LOG_INFO_STREAM("[ellipticCylinder::setAxisFromTangentVectorsProjections] Cylinder with length inferior to major ellipse axis !!!");
        m_principalBase.col(0).swap(m_principalBase.col(1));
    }
    else if(dotProds(2) < dotProds(0) && dotProds(2) < dotProds(1))
    {
        MRPT_LOG_INFO_STREAM("[ellipticCylinder::setAxisFromTangentVectorsProjections] Cylinder with length inferior to minor ellipse axis !!!");
        m_principalBase.col(0).swap(m_principalBase.col(2));
        m_principalBase.col(2).swap(m_principalBase.col(1));
    }

    // To ensure orthonormate DIRECT base, use cross-product
    // (Could have used the third eigenvector directly, but it can give indirect base :/)
    m_principalBase.col(2) = m_principalBase.col(0).cross(m_principalBase.col(1));
    m_principalBase_t = m_principalBase.transpose();
}

void ellipticCylinder::fitPCA_raw_grassmanAverages(const vector<CPointPDFGaussian>& points,
                                                   Eigen::Matrix3Xd& centered_pts)
{
    cout << "Points size : " << points.size() << endl;

    // Handy typedefs
    using namespace grassmann_averages_pca;
    using namespace grassmann_averages_pca::details::ublas_helpers;
    namespace ub = boost::numeric::ublas;
    using data_t = ub::vector<double>;
    using matrix_t = ub::matrix<double>;
    using grassmann_pca_t = grassmann_pca< data_t >;
    using const_row_iter_t = row_iter<const matrix_t>;

    Eigen::Vector3d tangent = Eigen::Vector3d::Zero(),
                    dotProds = Eigen::Vector3d::Zero(),
                    curDot = Eigen::Vector3d::Zero();
    int N = points.size();

    // Center of the data
    double x_mean = 0., y_mean = 0., z_mean = 0.;
    double inv_size = 1./(double)N;
    for(const CPointPDFGaussian& pt : points)
    {
        x_mean += pt.mean.x();
        y_mean += pt.mean.y();
        z_mean += pt.mean.z();
    }
    x_mean *= inv_size;
    y_mean *= inv_size;
    z_mean *= inv_size;

    // Fill the centered data matrix
    matrix_t data_pts;
    data_pts.resize(N,3);
    int i = 0;
    for(const CPointPDFGaussian& pt : points)
    {
        const CPoint3D& pt_mean = pt.mean;
        data_pts(i,0) = pt_mean.x() - x_mean;
        data_pts(i,1) = pt_mean.y() - y_mean;
        data_pts(i,2) = pt_mean.z() - z_mean;
        i++;
    }

    // Initial eigen vectors
    data_t v_init(3);
    v_init(0) = 1.;
    v_init(1) = 0.;
    v_init(2) = 0.;
    vector<data_t> v_init_points(3,v_init);

    // Output
    vector<data_t> basis_vectors(3);

    // Computation
    // BUG: The results varies at each run when using more than on thread ...
    // Errors on the basis vector are small (> 10^10) but it should be exactly the same
    // --> suspect some resource sharing problem in the grassmann_pca lib 
    // Here we work with a relatively small number of points so not a big deal to use monothread
    grassmann_pca_t rpca;
    if(!rpca.set_nb_processors(1)) //8
        MRPT_LOG_ERROR_STREAM("[ellipticCylinder::fitPCA_raw_grassmanAverages] Error while setting thread PCA");
    if(!rpca.set_nb_steps_pca(3))
        MRPT_LOG_ERROR_STREAM("[ellipticCylinder::fitPCA_raw_grassmanAverages] Error while setting the number of PCA steps");

    bool res = rpca.batch_process(
                300, // iterations
                3, // dimensions
                const_row_iter_t(data_pts,0),
                const_row_iter_t(data_pts,data_pts.size1()),
                basis_vectors.begin(),
                &v_init_points);

    if(!res)
        MRPT_LOG_ERROR_STREAM("[ellipticCylinder::fitPCA_raw_grassmanAverages] Error with robust PCA !!!!");

    // set Center
    m_pose.m_coords[0] = x_mean;
    m_pose.m_coords[1] = y_mean;
    m_pose.m_coords[2] = z_mean;

    // Set principal base
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
            m_principalBase(j,i) = basis_vectors[i](j);
    }

    cout << "m_principalBase_pca : " << std::setprecision(std::numeric_limits<long double>::digits10 + 1) << m_principalBase << endl;

    m_principalBase_t = m_principalBase.transpose();

    double x, y, z, prev_x, prev_y, prev_z;
    prev_x = data_pts(0,0);
    prev_y = data_pts(0,1);
    prev_z = data_pts(0,2);
    for(int r = 1; r < N; r++)
    {
         x = data_pts(r,0);
         y = data_pts(r,1);
         z = data_pts(r,2);

         tangent(0) = x - prev_x;
         tangent(1) = y - prev_y;
         tangent(2) = z - prev_z;

         curDot = m_principalBase_t*tangent;
         for(int i = 0; i < 3 ; i++)
            dotProds(i) += fabs(curDot(i));

         prev_x = x;
         prev_y = y;
         prev_z = z;
    }

    setAxisFromTangentVectorsProjections(dotProds);

    // Convert from matrix_t to Eigen::Matrix3Xd (Todo : avoid hard copy ?)
    centered_pts.resize(3,N);
    for(int r = 0; r < N; r++)
    {
        for(int c = 0; c < 3; c++)
            centered_pts(c,r) = data_pts(r,c);
    }
}

vector<cv::Point2f> ellipticCylinder::projectOnOrthogonalPlane(const Eigen::Matrix3Xd& centered_pts)
{
    // Length and axis half length
    //
    // Length :Found by projecting on the first base vector (largest eigenvalue) and getting
    // L = max(proj) - min(proj)
    //
    // Axis half lengths : Project on the plan spanned by the other two vectors of the base
    // Then, fit the 2D points by an ellipse

    int N = centered_pts.cols();

    // Output
    vector<cv::Point2f> projectedPtsOnPlane;
    projectedPtsOnPlane.reserve(N);

    // Points correspond to columns
    Eigen::Matrix3Xd projected_pt = m_principalBase_t*centered_pts;

    double max_projOnFirstVect = -1e8, min_projOnFirstVect = 1e8;
    for(int r = 0; r < N; r++)
    {
        if(projected_pt(0,r) > max_projOnFirstVect)
            max_projOnFirstVect = projected_pt(0,r);
        if(projected_pt(0,r) < min_projOnFirstVect)
            min_projOnFirstVect = projected_pt(0,r);

        projectedPtsOnPlane.push_back(cv::Point2f(projected_pt(1,r), projected_pt(2,r)));
    }

    m_length = max_projOnFirstVect - min_projOnFirstVect;

    return projectedPtsOnPlane;
}

void ellipticCylinder::fitPCA(const vector<CPointPDFGaussian>& points)
{
    // Prefered over the opencv version which is not robust
    Eigen::Matrix3Xd centered_pts;
    fitPCA_raw_grassmanAverages(points, centered_pts);

    // Project the points on the plane orthogonal to cylinder axis and compute length
    vector<cv::Point2f> projectedPtsOnPlane = projectOnOrthogonalPlane(centered_pts);

    cv::RotatedRect rect = cv::fitEllipseDirect(projectedPtsOnPlane);
    float ellipse_orientation = rect.angle*M_PIl/180.; // Opencv use angle in degree :)
    if(rect.size.width > rect.size.height)
    {
        m_halfLongAxis  = 0.5*rect.size.width;
        m_halfSmallAxis = 0.5*rect.size.height;
    }
    else
    {
        m_halfSmallAxis = 0.5*rect.size.width;
        m_halfLongAxis  = 0.5*rect.size.height;
        ellipse_orientation -= 0.5*M_PIl; // to keep y as the long axis
    }

    // Rotate around the first vector to have the other two basis vector along the ellipse axis
    float cs = cos(ellipse_orientation), ss = sin(ellipse_orientation);
    Eigen::Matrix3d R;
    R << 1., 0., 0.,
         0., cs, -ss,
         0., ss, cs;
    m_principalBase = m_principalBase*R;
    m_pose.setRotationMatrix(m_principalBase);
    m_principalBase_t = m_principalBase.transpose();

    // Keep pca data
    m_halfLongAxis_pca = m_halfLongAxis;
    m_halfSmallAxis_pca = m_halfSmallAxis;
    m_principalBase_pca = m_principalBase;
    m_principalBase_t_pca = m_principalBase_t;
    m_pose_pca = m_pose;
    m_length_pca = m_length;
}

void ellipticCylinder::incrementAdder_LM_infinite(CVectorDouble &newVals,
                                                  const CVectorDouble &oldVals,
                                                  const CVectorDouble &incr,
                                                  const auxData_fitNLLS &auxData)
{
    auxData_fitNLLS& auxData_ = const_cast<auxData_fitNLLS&>(auxData);
    if(!auxData.isImprovement)
    {
        CPose3D incr_SE3;
        CArrayNumeric<double,6> incr_se3;
        incr_se3 << incr[0],incr[1],incr[2],incr[3],incr[4],incr[5];
        CPose3D::exp(incr_se3, incr_SE3);

        auxData_.tpEstimatedPose  = auxData_.curEstimatedPose + incr_SE3;

        newVals.resize(8);
        for(int i = 0; i <= 5; i++)
            newVals[i] = 0.;
        newVals[6] = oldVals[6] + incr[6];
        newVals[7] = oldVals[7] + incr[7];
    }
    else
        auxData_.curEstimatedPose = move(auxData_.tpEstimatedPose);
}

void ellipticCylinder::incrementAdder_LM_finite(CVectorDouble &newVals,
                                                const CVectorDouble &oldVals,
                                                const CVectorDouble &incr,
                                                const auxData_fitNLLS &auxData)
{
    auxData_fitNLLS& auxData_ = const_cast<auxData_fitNLLS&>(auxData);
    if(!auxData.isImprovement)
    {
        CPose3D incr_SE3;
        CArrayNumeric<double,6> incr_se3;
        incr_se3 << incr[0],incr[1],incr[2],incr[3],incr[4],incr[5];
        CPose3D::exp(incr_se3, incr_SE3);

        auxData_.tpEstimatedPose  = auxData_.curEstimatedPose + incr_SE3;

        newVals.resize(9);
        for(int i = 0; i <= 5;i++)
            newVals[i] = 0.;
        newVals[6] = oldVals[6] + incr[6];
        newVals[7] = oldVals[7] + incr[7];
        newVals[8] = oldVals[8] + incr[8];
    }
    else
        auxData_.curEstimatedPose = move(auxData_.tpEstimatedPose);    
}

CPose3D ellipticCylinder::errorFunctionForLM_cylinderPose(const CVectorDouble& cylinderVals,
                                                          const auxData_fitNLLS& auxData)
{
    CPose3D expEps;
    CArrayNumeric<double,6> eps;
    eps << cylinderVals[0],cylinderVals[1],cylinderVals[2],cylinderVals[3],cylinderVals[4],cylinderVals[5];
    CPose3D::exp(eps, expEps);

    // Use the current estimation only if it improves the LM error function
    // Otherwise ignore it and use the previous "best" estimation
    CPose3D cylinderPose = auxData.isImprovement ? auxData.curEstimatedPose : auxData.tpEstimatedPose;
    cylinderPose += expEps;

    return cylinderPose;
}

void ellipticCylinder::errorFunctionForLM_finite(const CVectorDouble& cylinderVals,
                                                 const auxData_fitNLLS& auxData,
                                                 CVectorDouble& f)
{
    CPose3D cylinderPose = errorFunctionForLM_cylinderPose(cylinderVals, auxData);
    
    // Compute error
    f.resize(1);
    double error = 0., e;
    for(const CPointPDFGaussian& pt : auxData.points)
    {
        e = orthogonalDistanceToFiniteCylinder(cylinderPose, cylinderVals[6], cylinderVals[7] , cylinderVals[8], pt.mean);
        error += e*e;
    }
    f[0] = sqrt(error);
}

double ellipticCylinder::distanceToCylinderAxis(const CPose3D &cylinderPose, const CPose3D &sonarPose)
{
    Eigen::Vector3d x_axe = cylinderPose.getRotationMatrix().block<3,1>(0,0);
    Eigen::Vector3d crossProd = x_axe.cross(sonarPose.m_coords -  cylinderPose.m_coords);

    return crossProd.norm();
}

void ellipticCylinder::errorFunctionForLM_infinite_constrained(const CVectorDouble& cylinderVals,
                                                               const auxData_fitNLLS& auxData,
                                                               CVectorDouble& f)
{
    CPose3D cylinderPose = errorFunctionForLM_cylinderPose(cylinderVals, auxData);

    // Compute error
    f.resize(1);
    double error_ellipse = 0., error_pose = 0., e;
    for(const CPointPDFGaussian& pt : auxData.points)
    {
        e = orthogonalDistanceToInfiniteCylinder(cylinderPose, cylinderVals[6], cylinderVals[7], pt.mean);
        error_ellipse += e*e;
    }
    for(const CPose3D& sonarPose : auxData.sonarPoses)
    {
        e = distanceToCylinderAxis(cylinderPose, sonarPose);
        error_pose += e*e;
    }

    // Weight between the two errors
    double k = 0.2; //0.9;
    f[0] = sqrt((1.-k)*error_ellipse + k*error_pose);
}

void ellipticCylinder::errorFunctionForLM_infinite(const CVectorDouble& cylinderVals,
                                                   const auxData_fitNLLS& auxData,
                                                   CVectorDouble& f)
{
    CPose3D cylinderPose = errorFunctionForLM_cylinderPose(cylinderVals, auxData);

    // Compute error
    f.resize(1);
    double error = 0., e;
    for(const CPointPDFGaussian& pt : auxData.points)
    {
        e = orthogonalDistanceToInfiniteCylinder(cylinderPose, cylinderVals[6], cylinderVals[7], pt.mean);
        error += e*e;
    }
    f[0] = sqrt(error);
}

double ellipticCylinder::orthogonalDistanceToInfiniteCylinder(const CPose3D& cylinderPose,
                                                              double halfLongAxis,
                                                              double halfSmallAxis,
                                                              const CPoint3D& P)
{
    // Point in the cylinder local coords
    TPoint3D localP;
    cylinderPose.inverseComposePoint(P,localP);

    return distanceToEllipse(localP.y, localP.z, halfLongAxis, halfSmallAxis);
}

double ellipticCylinder::orthogonalDistanceToFiniteCylinder(const CPose3D& cylinderPose,
                                                            double halfLongAxis,
                                                            double halfSmallAxis,
                                                            double length,
                                                            const CPoint3D& P)
{
    double d;

    // Point in the cylinder local coords
    TPoint3D localP;
    cylinderPose.inverseComposePoint(P,localP);

    // First case : the point is between the two cylinder base plane
    if(fabs(localP.x) <= 0.5*length)
        d = orthogonalDistanceToInfiniteCylinder(cylinderPose, halfLongAxis, halfSmallAxis, P);
    // Second case : the point is "in front of"  or "behind" a base plane (ie inside the cylinder if it was infinite but outside the real finite cylinder)
    else if(fabs(localP.y) < halfLongAxis && fabs(localP.z) < halfSmallAxis)
        d = fabs(localP.x) - 0.5*length;
    // Third case : in between the previous cases. The distance is the distance to the nearest base ellipse.
    else
    {
        double distToBasePlane = fabs(localP.x) - 0.5*length;
        // Distance of the projected point in the base ellipse to the ellipse
        double d_plane = distanceToEllipse(localP.y, localP.z,
                                           halfLongAxis, halfSmallAxis);

        // Use pythagore to get the 3D distance
        d = sqrt(distToBasePlane*distToBasePlane + d_plane*d_plane);
    }

    return d;
}

double ellipticCylinder::getOrthogonalDistanceToFiniteCylinder(const CPoint3D& P) const
{
    return orthogonalDistanceToFiniteCylinder(m_pose, m_halfLongAxis, m_halfSmallAxis, m_length, P);
}

double ellipticCylinder::getOrthogonalDistanceToInfiniteCylinder(const CPoint3D& P) const
{
    return orthogonalDistanceToInfiniteCylinder(m_pose, m_halfLongAxis, m_halfSmallAxis, P);
}

double ellipticCylinder::getDistanceToInfiniteCylinderAlongDirection(const Eigen::Vector3d &P,
                                                                     const Eigen::Vector3d &dir) const
{
    // Project the points and the direction in the cylinder orthogonal plane
    Eigen::Vector2d P_projOnCylinderPlane   = m_principalBase_t.block<2,3>(1,0)*(P - m_pose.m_coords),
                    dir_projOnCylinderPlane = m_principalBase_t.block<2,3>(1,0)*dir;

    double inv_halfLongAxis_sqr  = 1./(m_halfLongAxis*m_halfLongAxis),
           inv_halfSmallAxis_sqr = 1./(m_halfSmallAxis*m_halfSmallAxis);

    double P_proj_y   = P_projOnCylinderPlane(0),
           P_proj_z   = P_projOnCylinderPlane(1),
           dir_proj_y = dir_projOnCylinderPlane(0),
           dir_proj_z = dir_projOnCylinderPlane(1);

    // Search for the intersection(s) (if any)
    double a = dir_proj_y*dir_proj_y*inv_halfLongAxis_sqr +
               dir_proj_z*dir_proj_z*inv_halfSmallAxis_sqr;
    double b = 2.*(P_proj_y*dir_proj_y*inv_halfLongAxis_sqr +
                   P_proj_z*dir_proj_z*inv_halfSmallAxis_sqr);
    double c = P_proj_y*P_proj_y*inv_halfLongAxis_sqr +
               P_proj_z*P_proj_z*inv_halfSmallAxis_sqr - 1.;

    double r1,r2;
    int nRoots = mrpt::math::solve_poly2(a,b,c,r1,r2);

    double res = -1;
    if(nRoots > 0)
    {
        if(r1 > 0)
            res = r1;
        else if(nRoots == 2 && r2 > 0)
            res = r2;
    }

    if(res == -1)
    {
        MRPT_LOG_ERROR_STREAM("[ellipticCylinder::getDistanceToInfiniteCylinderAlongDirection] No roots found ! ");
        MRPT_LOG_DEBUG_STREAM("---------------> Debug Info <--------------------------------");
        MRPT_LOG_DEBUG_STREAM("P inside cylinder ?  :" << isInsideEllipse(P_proj_y, P_proj_z, m_halfLongAxis, m_halfSmallAxis));
        MRPT_LOG_DEBUG_STREAM("[getDistanceToInfiniteCylinderAlongDirection] P, dir : " << P << "," << dir);
        MRPT_LOG_DEBUG_STREAM("[getDistanceToInfiniteCylinderAlongDirection] P_proj, dir_proj" << P_projOnCylinderPlane << "," << dir_projOnCylinderPlane);
        MRPT_LOG_DEBUG_STREAM("[getDistanceToInfiniteCylinderAlongDirection] r1, r2 : " << r1 << "," << r2);
        MRPT_LOG_DEBUG_STREAM("[getDistanceToInfiniteCylinderAlongDirection] a,b,c : " << a << "," << b << "," << c);
        MRPT_LOG_DEBUG_STREAM(" -------------------------------------------------------------");
    }

    return res;
}

Eigen::Vector3d ellipticCylinder::bottomBaseCenter() const
{
    return m_pose.m_coords - 0.5*m_length*m_principalBase.col(0);
}

Eigen::Vector3d ellipticCylinder::bottomBaseCenter_pca() const
{
    return m_pose_pca.m_coords - 0.5*m_length_pca*m_principalBase_pca.col(0);
}

void ellipticCylinder::setPose(const CPose3D &pose)
{
    m_pose = pose;
    setPrincipalBase(m_pose.getRotationMatrix());
}

void ellipticCylinder::setPrincipalBase(const Eigen::Matrix3d &R)
{
    m_principalBase = R;
    m_principalBase_t = R.transpose();
}

void ellipticCylinder::print() const
{
    cout << "Elliptic Cylinder Pose " << m_pose          << endl;
    cout << "Length     : "           << m_length        << endl;
    cout << "Long axis  : "           << m_halfLongAxis  << endl;
    cout << "Small axis : "           << m_halfSmallAxis << endl;
}

void ellipticCylinder::save(const string &fileName) const
{
    ofstream file(fileName);
    if(file.is_open())
    {
        file << "# Length_pca, Half long axis_pca, half small axis_pca, principalBaseMatrixVectorized_pca, center_pca, Length, ... \n";
        file << m_length_pca << "\n";
        file << m_halfLongAxis_pca << "\n";
        file << m_halfSmallAxis_pca << "\n";
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                file << m_principalBase_pca(i,j) << "\n";
        for(int i = 0; i < 3; i++)
            file << m_pose_pca.m_coords[i] << "\n";

        file << m_length << "\n";
        file << m_halfLongAxis << "\n";
        file << m_halfSmallAxis << "\n";
        for(int i = 0; i < 3; i++)
            for(int j = 0; j < 3; j++)
                file << m_principalBase(i,j) << "\n";
        for(int i = 0; i < 3; i++)
            file << m_pose.m_coords[i] << "\n";

        file.close();
    }
}

void ellipticCylinder::load(const string &fileName)
{
    ifstream file(fileName);
    string line;
    if(file.is_open())
    {
        getline(file,line); // skip header


        getline(file,line);
        m_length_pca = stod(line);

        getline(file,line);
        m_halfLongAxis_pca = stod(line);

        getline(file,line);
        m_halfSmallAxis_pca = stod(line);

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                getline(file,line);
                m_principalBase_pca(i,j) = stod(line);
            }
        }
        m_principalBase_t_pca = m_principalBase_pca.transpose();

        m_pose_pca.setRotationMatrix(m_principalBase_pca);
        for(int i = 0; i < 3; i++)
        {
            getline(file,line);
            m_pose_pca.m_coords[i] = stod(line);
        }

        getline(file,line);
        m_length = stod(line);

        getline(file,line);
        m_halfLongAxis = stod(line);

        getline(file,line);
        m_halfSmallAxis = stod(line);

        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                getline(file,line);
                m_principalBase(i,j) = stod(line);
            }
        }

        m_principalBase_t = m_principalBase.transpose();

        m_pose.setRotationMatrix(m_principalBase);
        for(int i = 0; i < 3; i++)
        {
            getline(file,line);
            m_pose.m_coords[i] = stod(line);
        }

        file.close();
    }
    else
        MRPT_LOG_ERROR_STREAM("Can't open file " << fileName);
}

bool ellipticCylinder::equalTo(const ellipticCylinder &other,
                               double eps) const
{
    return karst_slam::utils::equalPose(m_pose, other.getPose(), eps) &&
           fabs(m_halfLongAxis - other.getHalfLongAxis()) < eps &&
           fabs(m_halfLongAxis - other.getHalfSmallAxis()) < eps &&
           fabs(m_length - other.getLength()) < eps;
}

// ToDo : Precompute values used for each distance computation (a_sqr etc...)
double ellipticCylinder::distanceToEllipse(double point_x, double point_y,
                                           double a, double b, int nIterations)
{
    double px = fabs(point_x),
           py = fabs(point_y);

    double tx = txy,
           ty = txy,
           tx_sqr,
           ty_sqr;

    double x, y, ex, ey, rx, ry, qx, qy, r, q, t = 0;

    double a_sqr = a*a, b_sqr = b*b, inv_a = 1./a, inv_b = 1./b;

    for(int i = 0; i < nIterations; i++)
    {
        tx_sqr = tx*tx;
        ty_sqr = ty*ty;

        x = a * tx;
        y = b * ty;

        ex = (a_sqr - b_sqr) * (tx_sqr * tx) * inv_a;
        ey = (b_sqr - a_sqr) * (ty_sqr * ty) * inv_b;

        rx = x - ex;
        ry = y - ey;

        qx = px - ex;
        qy = py - ey;

        r = sqrt(rx * rx + ry * ry);
        q = sqrt(qy * qy + qx * qx);

        tx = std::min(1., std::max(0., (qx * r / q + ex) * inv_a));
        ty = std::min(1., std::max(0., (qy * r / q + ey) * inv_b));

        t = sqrt(tx_sqr + ty_sqr);

        tx /= t;
        ty /= t;
    }

    x = a * (point_x < 0 ? -tx : tx);
    y = b * (point_y < 0 ? -ty : ty);

    double dx = x - point_x, dy = y - point_y;
    return sqrt(dx*dx + dy*dy);
}
