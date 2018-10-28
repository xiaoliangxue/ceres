//
// Created by chen on 18-7-18.
//

#ifndef LOCALIZER_LOCALIZER_H
#define LOCALIZER_LOCALIZER_H


#include <global.hpp>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <boost/algorithm/string.hpp>

#include <proj_api.h>

#include <eigen3/Eigen/Dense>
#include "AnnPointsVector2D.hpp"
#include "flags.h"
#include "Particle.hpp"
#include "RowGrids.hpp"
#include "SyncEKF.hpp"
#include "Wgs84Utils.hpp"

using namespace Eigen;
using namespace std;

struct GNSS_INFO
{
    double fTimeStamp;
    double fLatitude;
    double fLongitude;
    double fAltitude;

    double fHeading;
    double fVelEast;
    double fVelNorth;

    double fEngX;
    double fEngY;

    tf::Quaternion stQuat;

    bool bVelocityInitialized;
    bool bHeadingInitialized;
    bool bPositionInitialized;
};

class Localizer
{
public:
    Localizer();

    ~Localizer();


private:
    ros::NodeHandlePtr m_pNodeHandle;

    ros::Subscriber m_subMiddleLidar;

    ros::Subscriber m_subGnssHeading;

    ros::Subscriber m_subGnssPosition;

    ros::Subscriber m_subGnssVelocity;

    ros::Subscriber m_subImu;

    ros::Subscriber m_subWheelSpeed;

    ros::Subscriber m_subMapEdgePts;

    ros::Subscriber m_subMapPolePts;

    ros::Subscriber m_subMapMarkerPts;

    AnnPointsVector2D<PointXYZ> m_stMapEdgePts;

    AnnPointsVector2D<PointXYZ> m_stMapMarkerPts;

    AnnPointsVector2D<PointXYZ> m_stMapPolePts;

    ros::Timer m_stPredictTimer;

    Matrix4d m_emECEF2Eng;//transform matrix from ecef to engineer frame;

    Matrix4d m_emEng2ECEF;//transform matrix from engineer to ecef;
    projPJ m_stProjWGS84;

    projPJ m_stProjECEF;

    GNSS_INFO m_stGnssInfo;

    boost::shared_mutex m_mtxGnssInfo;

    double m_fGpsStdVar;//standard variance of the gps;
    double m_fHeadingStdVar;//standard variance of the gnss heading;
    double m_fWheelSpeedStdVar;//standard variance of the wheel speed;
    double m_fYawVeloStdVar;//standard variance of the yaw velocity;

    vector<Particle> m_vecParticles;

    bool m_bParticlesInitialized;

    boost::shared_mutex m_mtxParticles;

    double m_fWheelSpeed;

    double m_fYawVelocity;

    boost::shared_mutex m_mtxWheelSpeed;

    boost::shared_mutex m_mtxYawVelocity;

    boost::shared_mutex m_mtxMapFeatures;

    bool m_bUsePolePts;

    bool m_bUseEdgePts;

    bool m_bUseMarkerPts;

    SyncEKF m_stEKF;

private:
    void CallbackMiddleRSLidar(const sensor_msgs::PointCloud2ConstPtr &msg);

    void CallbackGnssPosition(const sensor_msgs::NavSatFixConstPtr &msg);

    void CallbackGnssHeading(const geometry_msgs::TwistStampedConstPtr &msg);

    void CallbackGnssVelocity(const geometry_msgs::TwistStampedConstPtr &msg);

    void CallbackImu(const sensor_msgs::ImuConstPtr &msg);

    void CallbackPredictTimer(const ros::TimerEvent &event);

    void CallbackWheelSpeed(const nav_msgs::OdometryConstPtr &msg);

    void CallbackMapEdgePts(const sensor_msgs::PointCloudConstPtr &msg);

    void CallbackMapPolePts(const sensor_msgs::PointCloudConstPtr &msg);

    void CallbackMapMarkerPts(const sensor_msgs::PointCloudConstPtr &msg);

    void ExtractPoles(const sensor_msgs::PointCloud &scan, PointsVector<PointXYZ> &poles);

    void ExtractEdgesAndMarkers(const sensor_msgs::PointCloud &scan, PointsVector<PointXYZ> &edges, PointsVector<PointXYZI> &markers);

    bool IsPolePts(PointsVector<PointXYZ> &pts, PointXYZ &center_pt);

    bool LoadTransform(const string &str);

    void TransformWGS84ToEngineer(double latitude, double longitude, double altitude, double &ex, double &ey, double &ez);

    void TransformEngineerToWGS84(double ex, double ey, double ez, double& latitude, double& longitude, double& altitude);

    void TransformEngineerToUTM(double ex, double ey, double ez, double& ux, double& uy, double& uz);

    void Quaternion2Euler(const tf::Quaternion &quat, double &yaw, double &pitch, double &roll);

    void SampleGpsParticles();

    void InitializeParticles(double gps_x, double gps_y, double heading);

    void ResampleParticles();

    void CalcHeading(double vx, double vy, double &heading);

    void ICPMatch(const PointsVector<PointXYZ> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, const Vector3d &init_pose, Vector3d &result_pose);

    void ICP(const PointsVector<PointXYZ> &source_pts, const PointsVector<PointXYZ> &target_pts, double threshold, tf::Transform &tf);

    void TransformPts(const tf::Transform &tf, const PointsVector<PointXYZ> &in, PointsVector<PointXYZ> &out);

    void UpdateParticles(const ros::Time& timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, Vector3d &best_pose);

    void EvaluateParticle(const PointsVector<PointXYZ> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, Particle &particle);

    void EvaluateParticles(const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts);

    void PublishLocalizedFeaturePts(const ros::Time &timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, const Vector3d &pose);

    void PublishDetectedFeaturePts(const ros::Time &timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts);

    void PublishGpsTrajectory(const ros::Time &timestamp, double gps_x, double gps_y);

    void PublishParticleTrajectory(const ros::Time &timestamp, const Vector3d &pose);

    void PublishPriorTrajectory(const ros::Time &timestamp, const Eigen::Matrix<double, 5, 1> &pose);

    void PublishPosteriorTrajectory(const ros::Time &timestamp, const Eigen::Matrix<double, 5, 1> &pose);

    void PublishParticles(const ros::Time &timestamp);

    void PublishLocalizedPose(const ros::Time &stamp, const Vector3d &pose);

    void PublishPriorPose(const ros::Time &stamp, const Vector4d& pose);

    bool LoadMapFeaturePts();

    bool LoadMapFeaturePts(const string &path, AnnPointsVector2D<PointXYZ> &pts);

    void NormalizeYaw(double &yaw)
    {
        if (yaw < 0.0)
        {
            yaw = PI_DOUBLE + yaw;
        }

        if (yaw > PI_DOUBLE)
        {
            yaw -= PI_DOUBLE;
        }
    }
};


#endif //LOCALIZER_LOCALIZER_H
