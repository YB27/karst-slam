#ifndef OBSERVATIONMSISBEAM_H
#define OBSERVATIONMSISBEAM_H

#include "karst_slam/typedefs.h"

#include <mrpt/utils/CSerializable.h>
#include <mrpt/obs/CObservation.h>
#include "karst_slam/mrpt_poses_frwd.h"

namespace karst_slam{namespace obs{

/** Represent a single scan measure (beam) with uncertainty */
struct beamPoints
{
    beamPoints() = default;

    beamPoints(int nPoint)
    {
        points = Eigen::Matrix<double, 4, Eigen::Dynamic,Eigen::RowMajor>(4,nPoint);
        cov.resize(nPoint);
    }

    beamPoints(const beamPoints& sp)
    {
       points = sp.points;
       cov    = sp.cov;
    }

    void clear()
    {
        points.resize(0,0);
        cov.clear();
        cov.shrink_to_fit();
    }

    points_mat_t points;
    eigenAlignedVector<mrpt::math::CMatrixDouble33> cov;
};

DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( ObservationMSISBeam, CObservation,OBS_IMPEXP )
class OBS_IMPEXP ObservationMSISBeam : public mrpt::obs::CObservation
{
    DEFINE_SERIALIZABLE( ObservationMSISBeam )

public:
    static ObservationMSISBeamPtr Create(const std::string& deviceName);

    ObservationMSISBeam() = default;
    explicit ObservationMSISBeam(const std::string& deviceName);
    virtual ~ObservationMSISBeam() = default;

    void getDescriptionAsText(std::ostream& o) const override;

    // Pose of the sonar in the body frame
    void getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const override 
    {out_sensorPose = m_sonarPoseInBodyFrame;}
    void setSensorPose(const mrpt::poses::CPose3D& newSensorPose) override 
    {m_sonarPoseInBodyFrame = newSensorPose;}

    // ToDo : The relative pose is not really part of the beam observation ....
    // Maybe have a structure which manage the beams and their relative pose until a full scan finished
    inline void setAngle(double angle){m_angle = angle;}
    inline void setIntensities(const std::vector<int>& intensities)
    {m_intensities = intensities;}
    inline void setIntensities(std::vector<int>&& intensities)
    {m_intensities = intensities;}
    inline void setNAngularStep(int n){m_nAngularStep = n;}
    inline void setMaxRange(int maxRange){m_maxRange = maxRange;}

    //inline bool hasPose()const {return m_hasPose;}
    inline double getAngle() const {return m_angle;}
    inline const std::vector<int>& getIntensities() const {return m_intensities;}
    inline int getNAngularStep()const{return m_nAngularStep;}
    inline int getMaxRange()const{return m_maxRange;}

protected:
    mrpt::poses::CPose3D m_sonarPoseInBodyFrame; //!< Offset pose relative to the body frame of the sonar from which this beam comes from
    double m_angle; //!< Scan rotation angle at measure time (in radian)
    std::vector<int> m_intensities; //!< Vector of intensities for each range bin (see the sonar doc for more details)
    int m_nAngularStep; // Number of beam to have a complete sector
    int m_maxRange; //!< Maximum range of the sonar which measured this beam 
};
DEFINE_SERIALIZABLE_POST_CUSTOM_BASE_LINKAGE( ObservationMSISBeam, mrpt::obs::CObservation,OBS_IMPEXP )
}} // end namespaces

#endif // OBSERVATIONMSISBEAM_H
