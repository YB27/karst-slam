#ifndef DEF_SURFACE_DATUM_H
#define DEF_SURFACE_DATUM_H

#include <vector>
#include <mrpt/poses/CPose3D.h>
#include "karst_slam/scanMerging/rangeMapping.h"

namespace karst_slam{namespace scanMerging{

/**
 * This structure represents a zero-beam-width sonar measure as input and output of the function to be estimated by the
 * gaussian process (range = f(curvilinearAbscissa, yaw)). It consists of the curvilinear abcissa on the trajectory, the sonar yaw angle and the range
 */
struct surfaceDatum
{
    surfaceDatum() = default;
    surfaceDatum(const double s_,
                const double yaw_,
                const double out_r_)
    {
        s     = s_;
        yaw   = yaw_;
        out_r = out_r_;
    }
    surfaceDatum(const double s_,
                 const double yaw_,
                 const double r_prior_,
                 const double out_r_)
    {
        s       = s_;
        yaw     = yaw_;
        r_prior = r_prior_;
        out_r   = out_r_;
    }

    uint64_t timeStamp;
    double s; //!< Curvilinear abscissa
    double yaw; //!< Yaw angle (rad)
    double out_r; //!< Range - priorRange
    double r_prior; //!< Prior range (distance to the prior cylinder)
    bool isCensored = false; //!< Censored data ? (eg. in the case of sonar an off-range data)
    mrpt::math::TPoint3D pt; //!< point in cartesian coords
};

/** Values used for estimated normals (discrete derivative) 
 *  All values are computed around a measure made at time "t" when robot is at curvilinear abscissa s(t)
 * and sonar rotation angle at yaw(t)
*/
struct aux_normalData
{
    aux_normalData() = default;
    aux_normalData(const mrpt::poses::CPose3D& interp_pose_dt_plus_sonar_,
                   const mrpt::poses::CPose3D& interp_pose_dt_minus_sonar_,
                   const mrpt::poses::CPose3D& interp_pose_sonar_,
                   double s_dt_plus_,
                   double s_dt_minus_,
                   double yaw_dt_plus_,
                   double yaw_dt_minus_,
                   double r_prior_dt_plus_,
                   double r_prior_dt_minus_,
                   double r_prior_dyaw_plus_,
                   double r_prior_dyaw_minus_)
    {
        interp_pose_dt_plus_sonar = interp_pose_dt_plus_sonar_;
        interp_pose_dt_minus_sonar = interp_pose_dt_minus_sonar_;
        interp_pose_sonar = interp_pose_sonar_;
        s_dt_plus = s_dt_plus_;
        s_dt_minus = s_dt_minus_;
        yaw_dt_plus = yaw_dt_plus_;
        yaw_dt_minus = yaw_dt_minus_;
        r_prior_dt_plus = r_prior_dt_plus_;
        r_prior_dt_minus = r_prior_dt_minus_;
        r_prior_dyaw_minus = r_prior_dyaw_minus_;
        r_prior_dyaw_plus = r_prior_dyaw_plus_;
    }
    mrpt::poses::CPose3D interp_pose_dt_plus_sonar; //!< interpolated robot pose at s(t) + ds  where s is the curvilinear abscissa and t the current timestamp
    mrpt::poses::CPose3D interp_pose_dt_minus_sonar; // !< interpolated robot pose at s(t) - ds where s is the curvilinear abscissa and t the current timestamp
    mrpt::poses::CPose3D interp_pose_sonar;//!< interpolated robot pose at s(t)
    double s_dt_plus; //!< curvilinear abscissa at s(t) + ds
    double s_dt_minus; //!< curvilinear abscissa at s(t) - ds
    double yaw_dt_plus; //!< sonar rotation angle yaw(t) + dyaw
    double yaw_dt_minus; //!< sonar rotation angle yaw(t) - dyaw  
    double r_prior_dt_plus; //!< estimated range rho^v by the Gaussian Process for the input (s(t) + ds, yaw(t))
    double r_prior_dt_minus; //!< estimated range rho^v by the Gaussian Process for the input (s(t) - ds, yaw(t)) 
    double r_prior_dyaw_plus; //!< estimated range rho^v by the Gaussian Process for the input (s(t), yaw(t) + dyaw)
    double r_prior_dyaw_minus; //!< estimated range rho^v by the Gaussian Process for the input (s(t), yaw(t) - dyaw)
};

/**
 * This structure represents a possible sonar measure (as for surfaceDatum) but with non-negligeable beam width, taking in account for a pitch angle (theta).
 * Note that the values here are for a fixed pitch angle. See surfaceDatum_arcEstimation for the representation of the full beam.
 */
struct surfaceDatum_valueAtTheta
{
    surfaceDatum_valueAtTheta() = default;
    surfaceDatum_valueAtTheta(const surfaceDatum& sd_,
                              const double theta_,
                              const aux_normalData& normalData_)
    {
        sd         = sd_;
        theta      = theta_;
        normalData = normalData_;
    }

    surfaceDatum sd; //!< contains curvilinear abscissa, yaw angle and range
    double theta; //!< Pitch angle (rad)

    aux_normalData normalData; //!< Values used for estimated normals
};

/** This structure represents a sonar measure with a non-zero beam width. */
struct surfaceDatum_arcEstimation
{
    inline void reserve(size_t n)
    {
        dataPerTheta.reserve(n);
    }

    std::vector<surfaceDatum_valueAtTheta> dataPerTheta; //!< Vector of data for each sampled pitch angle values
    int idx; //!< Index of the arc
};


}} // end namespaces
#endif // DEF_SURFACE_DATUM_H
