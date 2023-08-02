#ifndef COMPARE_H
#define COMPARE_H

#include <mrpt/poses/CPose3DPDFGaussian.h>

namespace karst_slam{namespace utils{

/** Check if the matrix contains nan values */
template<class T>
bool isValidMatrix(const T& A)
{
    for(int i = 0; i < A.rows(); i++)
        for(int j = 0; j < A.cols(); j++)
        {
            if(std::isnan(A.coeff(i,j)))
                return false;
        }

    return true;
}

/** Check if two matrices are equal (up to a given tolerance) 
 * 
 * @param A n X m matrix
 * @param B n' X m' matrix
 * @param eps tolerance
 */
template<class T>
bool equalMatrix(const T& A,
                 const T& B,
                 double eps = 1e-5)
{
    // First check matrices dimensions
    ASSERT_(A.rows() == B.rows() && A.cols() == B.cols());

    // If one matrix is invalid, stop
    if(!isValidMatrix(A) || !isValidMatrix(B))
        return false;

    for(int i = 0; i < A.rows(); i++)
        for(int j = 0; j < A.cols(); j++)
        {
            if( fabs(A.coeff(i,j) - B.coeff(i,j)) > eps)
                return false;
        }
    return true;
}

/** Check if two 3D/2D poses are equal
 *  Template class T should derived from mrpt::poses::CPose 
 *
 * @param p1 first pose
 * @param p2 second pose
 * @param eps tolerance
 */
template<class T>
bool equalPose(const T& p1,
               const T& p2,
               double eps = 1e-5)
{
    return equalMatrix(p1.getHomogeneousMatrixVal(), p2.getHomogeneousMatrixVal(),eps);
}

/** Check that two 2D/3D poses pdf (probability distribution function) are equal
 * eg. same mean and covariance  
 * Template class T should derived from mrpt::poses::CPosePdf
 */ 
template<class T>
bool equalPosePdf(const T& p1,
                  const T& p2)
{
    return (equalMatrix(p1.getMeanVal().getHomogeneousMatrixVal(), p2.getMeanVal().getHomogeneousMatrixVal())
            && equalMatrix(p1.getCovariance(), p2.getCovariance()));
}
}} // end namespaces
#endif // COMPARE_H
