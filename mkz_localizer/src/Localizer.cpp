//
// Created by chen on 18-7-18.
//

#include "Localizer.h"


Localizer::Localizer()
    : m_pNodeHandle{new ros::NodeHandle},
      m_stProjWGS84{pj_init_plus("+proj=latlong +datum=WGS84 +units=m")},
      m_stProjECEF{pj_init_plus("+proj=geocent +datum=WGS84")},
      m_fGpsStdVar{FLAGS_std_var_gnss},
      m_fHeadingStdVar{FLAGS_std_var_heading},
      m_fWheelSpeedStdVar{FLAGS_std_var_wheel_speed},
      m_fYawVeloStdVar{FLAGS_std_var_yaw_velocity},
      m_bParticlesInitialized{false},
      m_bUseEdgePts{false},
      m_bUsePolePts{false},
      m_bUseMarkerPts{false}
{
    m_stGnssInfo.bHeadingInitialized = false;
    m_stGnssInfo.bPositionInitialized = false;
    m_stGnssInfo.bVelocityInitialized = false;

    LoadMapFeaturePts();
    LoadTransform(FLAGS_transform);

    m_stPredictTimer = m_pNodeHandle->createTimer(ros::Duration(0.025), &Localizer::CallbackPredictTimer, this);

    m_subMiddleLidar = m_pNodeHandle->subscribe("/middle/rslidar_points", 10, &Localizer::CallbackMiddleRSLidar, this);

    m_subGnssHeading = m_pNodeHandle->subscribe("/autogo/sensors/gnss/heading", 10, &Localizer::CallbackGnssHeading, this);

    m_subGnssPosition = m_pNodeHandle->subscribe("/autogo/sensors/gnss/position", 10, &Localizer::CallbackGnssPosition, this);

    m_subGnssVelocity = m_pNodeHandle->subscribe("/autogo/sensors/gnss/velocity", 10, &Localizer::CallbackGnssVelocity, this);

    m_subImu = m_pNodeHandle->subscribe("/autogo/sensors/imu", 10, &Localizer::CallbackImu, this);

    m_subWheelSpeed = m_pNodeHandle->subscribe("/vehicle/dbw_odometry", 10, &Localizer::CallbackWheelSpeed, this);

    m_subMapEdgePts = m_pNodeHandle->subscribe("/local_edges", 10, &Localizer::CallbackMapEdgePts, this);

    m_subMapMarkerPts = m_pNodeHandle->subscribe("/local_markers", 10, &Localizer::CallbackMapMarkerPts, this);

    m_subMapPolePts = m_pNodeHandle->subscribe("/local_poles", 10, &Localizer::CallbackMapPolePts, this);
}

Localizer::~Localizer()
{

}

/**@brief extract edges;
 *
 * 1. split each ring into 4 part, front left, front right, rear left, rear right;
 * 2. get change point in each part;
 * @param scan [IN]: the rslidar scan;
 * @param edges [OUT]: edges detected in the scan;
 */
void Localizer::ExtractEdgesAndMarkers(const sensor_msgs::PointCloud &scan, PointsVector<PointXYZ> &edges, PointsVector<PointXYZI> &markers)
{
    for (int r = 0; r < 32; ++r)
    {
        PointsVector<PointXYZI> front_left, front_right, rear_left, rear_right;
        for (int p = 0; p < 2028; ++p)
        {
            auto idx = r * 2028 + p;
            auto &pt = scan.points[idx];
            if (isnan(pt.x) || isnan(pt.y) || isnan(pt.z))
            {
                continue;
            }

            if (pt.z > -0.6 || (pt.x > -2.0 && pt.x < 2.0 && pt.y > -2.0 && pt.y < 2.0) || (pt.x * pt.x + pt.y * pt.y) > 3600)
            {
                continue;
            }

            if (pt.y > -20.0 && pt.y < 20.0)
            {
                if (pt.x > 0.0)
                {
                    if (pt.y > 0.0)
                    {
                        front_left.emplace_back(PointXYZI{pt.x, pt.y, pt.z + 1.8, scan.channels[0].values[idx]});
                    }
                    else
                    {
                        front_right.emplace_back(PointXYZI{pt.x, pt.y, pt.z + 1.8, scan.channels[0].values[idx]});
                    }
                }
                else
                {
                    if (pt.y > 0.0)
                    {
                        rear_left.emplace_back(PointXYZI{pt.x, pt.y, pt.z + 1.8, scan.channels[0].values[idx]});
                    }
                    else
                    {
                        rear_right.emplace_back(PointXYZI{pt.x, pt.y, pt.z + 1.8, scan.channels[0].values[idx]});
                    }
                }
            }
        }


        RowGrids front_left_grids, front_right_grids, rear_left_grids, rear_right_grids;
        front_left_grids.SetInputPoints(20.0, front_left);
        front_right_grids.SetInputPoints(-20.0, front_right);
        rear_left_grids.SetInputPoints(20.0, rear_left);
        rear_right_grids.SetInputPoints(-20.0, rear_right);

        PointXYZ pt;
        if (front_left_grids.GetEdgeAndMarkerPoints(pt, markers))
        {
            edges.emplace_back(pt);
        }
        if (front_right_grids.GetEdgeAndMarkerPoints(pt, markers))
        {
            edges.emplace_back(pt);
        }
        if (rear_left_grids.GetEdgeAndMarkerPoints(pt, markers))
        {
            edges.emplace_back(pt);
        }

        if (rear_right_grids.GetEdgeAndMarkerPoints(pt, markers))
        {
            edges.emplace_back(pt);
        }
    }
}

union GRID_INDEX
{
    uint64_t nGridIdx;
    struct GRID_POSITION
    {
        int32_t nRowIdx;
        int32_t nColIdx;
    } stGridPostion;
};

/**@brief extract pole points;
 *
 * extract pole points from the scan;
 * 1. push points into grid map;
 * 2. extract poles from the map;
 * @param scan [IN]: the rslidar scan;
 * @param poles [OUT]: the pole points;
 */
void Localizer::ExtractPoles(const sensor_msgs::PointCloud &scan, PointsVector<PointXYZ> &poles)
{
    unordered_map<uint64_t, PointsVector<PointXYZ>> grid_map;

    GRID_INDEX idx;
    for (int i = 0; i < scan.points.size(); ++i)
    {
        if (isnan(scan.points[i].x) || isnan(scan.points[i].y) || isnan(scan.points[i].z))
        {
            continue;
        }

        auto z = scan.points[i].z + 1.85;
        if (scan.points[i].x > -60.0 && scan.points[i].x < 60.0 && scan.points[i].y > -60.0 && scan.points[i].y < 60.0 && z > 0.5 && z < 2.5)
        {
            idx.stGridPostion.nRowIdx = int32_t(scan.points[i].x * 10.0);
            idx.stGridPostion.nColIdx = int32_t(scan.points[i].y * 10.0);

            grid_map[idx.nGridIdx].emplace_back(PointXYZ(scan.points[i].x, scan.points[i].y, z));
        }
    }

    for (auto grid:grid_map)
    {
        PointXYZ center_pt;
        if (IsPolePts(grid.second, center_pt))
        {
            poles.emplace_back(center_pt);
        }
    }
}

/**@brief callback function to localize by rslidar;
 *
 * callback function to localize by rslidar, main localization loop;
 * 1. extract edge and pole points;
 * 2. update particles;
 * 3. publish results;
 * @param msg [IN]: the rslidar scan;
 */
void Localizer::CallbackMiddleRSLidar(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (!m_bParticlesInitialized)
    {
        return;
    }

    double t0 = GetLocalTimeStampInSec();
    //convert point-cloud2 to point cloud;
    sensor_msgs::PointCloud scan;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, scan);

    double t1 = GetLocalTimeStampInSec();
    //extract pole feature points;
    PointsVector<PointXYZ> pole_pts;
    ExtractPoles(scan, pole_pts);

    //extract edge feature points;
    PointsVector<PointXYZ> edge_pts;
    PointsVector<PointXYZI> marker_pts;
    ExtractEdgesAndMarkers(scan, edge_pts, marker_pts);

    auto num_of_features = ((m_bUseMarkerPts && FLAGS_use_markers) ? marker_pts.size() : 0) + (m_bUseEdgePts ? edge_pts.size() : 0) + (m_bUsePolePts ? pole_pts.size() : 0);
    if (num_of_features < 10)
    {
        LOG(WARNING) << "not enough feature, cannot update. markers:" << marker_pts.size() << ", edges:" << edge_pts.size() << ", poles:" << pole_pts.size();
        return;
    }
    if (marker_pts.size() > 200)
    {
        marker_pts.resize(200);
    }

    if (edge_pts.size() > 100)
    {
        edge_pts.resize(100);
    }

    if (pole_pts.size() > 100)
    {
        pole_pts.resize(100);
    }

    double t2 = GetLocalTimeStampInSec();

    Eigen::Vector3d best_pose;
    UpdateParticles(msg->header.stamp, marker_pts, edge_pts, pole_pts, best_pose);

    double t3 = GetLocalTimeStampInSec();
    Eigen::Matrix<double, 5, 1> ekf_pose;
    m_stEKF.GetPose(ekf_pose);

    if (FLAGS_debug)
    {
        PublishDetectedFeaturePts(msg->header.stamp, marker_pts, edge_pts, pole_pts);
        PublishLocalizedFeaturePts(msg->header.stamp, marker_pts, edge_pts, pole_pts, best_pose);
        PublishParticles(msg->header.stamp);
        PublishParticleTrajectory(msg->header.stamp, best_pose);
    }

    PublishPosteriorTrajectory(msg->header.stamp, ekf_pose);
    //    PublishLocalizedPose(msg->header.stamp, best_pose);

    double t4 = GetLocalTimeStampInSec();
    LOG(INFO) << "localizing scan " << msg->header.seq << ", markers:" << marker_pts.size() << ", edges:" << edge_pts.size() << ", poles:" << pole_pts.size() << ", convert time:" << t1 - t0 << ", feature time:" << t2 - t1 << ", localization time:" << t3 - t2 << ", publish time:" << t4 - t3 << ", total time:" << t4 - t0;

}

/**@brief transform points by tf;
 *
 * transform points by tf;
 * @param tf [IN]: the tf used to transform the points;
 * @param in [IN]: the points to be transformed;
 * @param out [OUT]: the transformed points;
 */
void Localizer::TransformPts(const tf::Transform &tf, const PointsVector<PointXYZ> &in, PointsVector<PointXYZ> &out)
{
    out.reserve(in.size());
    for (const auto &pt:in)
    {
        auto tf_pt = tf * tf::Vector3{pt.x(), pt.y(), pt.z()};
        out.emplace_back(PointXYZ{tf_pt.x(), tf_pt.y(), tf_pt.z()});
    }
}

/**@brief check whether the points has pole or not;
 *
 * check whether the points has pole or not;
 * @param pts [IN]: the points in one grid;
 * @param center_pt [OUT]: the center of the pole points;
 * @return if there is pole in the points, return true; else, return false;
 */
bool Localizer::IsPolePts(PointsVector<PointXYZ> &pts, PointXYZ &center_pt)
{
    if (pts.size() < 10)
    {
        return false;
    }

    auto lowest_pt = pts.front();
    auto highest_pt = pts.front();
    for (auto &pt:pts)
    {
        if (pt.z() < lowest_pt.z())
        {
            lowest_pt = pt;
        }

        if (pt.z() > highest_pt.z())
        {
            highest_pt = pt;
        }
    }

    if (lowest_pt.z() > 1.2 || highest_pt.z() < 2.0)
    {
        return false;
    }

    center_pt = (lowest_pt + highest_pt) * 0.5;
    center_pt.z() = 0.0;

    return true;
}

/**@brief load map feature points from asc file;
 *
 * @param path [IN]: the absolute path to the asc file;
 * @param ann_pts [OUT]: the feature points;
 * @return if the points are successfully loaded, return true; else, return false;
 */
bool Localizer::LoadMapFeaturePts(const string &path, AnnPointsVector2D<PointXYZ> &ann_pts)
{
    FILE *fp = fopen(path.data(), "r");
    LOG_IF(FATAL, nullptr == fp) << "cannot load map file " << path;

    PointXYZI pt;
    while (1)
    {
        auto ret = fscanf(fp, "%lf\t%lf\t%lf\t%f\n", &(pt.x()), &(pt.y()), &(pt.z()), &(pt.i()));
        if (ret != 4)
        {
            break;
        }

        ann_pts.m_stPts.emplace_back(PointXYZ{pt.x(), pt.y(), pt.z()});
    }

    if (!ann_pts.m_stPts.empty())
    {
        ann_pts.BuildFLANNIndex();
    }
}

/**@brief load global map feature points;
 *
 * load global map feature points from asc files;
 * @return if the map asc files are successfully loaded, return true; else, return false;
 * @note the absolute path the map asc files are specified by prefix + edges.asc/markers.asc/poles.asc;
 * @note the global map maybe updated when local map features are received from patcher;
 */
bool Localizer::LoadMapFeaturePts()
{
    m_mtxMapFeatures.lock();
    LoadMapFeaturePts(FLAGS_map_prefix + string("edges.asc"), m_stMapEdgePts);
    LoadMapFeaturePts(FLAGS_map_prefix + string("markers.asc"), m_stMapMarkerPts);
    LoadMapFeaturePts(FLAGS_map_prefix + string("poles.asc"), m_stMapPolePts);

    m_bUseMarkerPts = !m_stMapMarkerPts.m_stPts.empty();
    m_bUsePolePts = !m_stMapPolePts.m_stPts.empty();
    m_bUseEdgePts = !m_stMapEdgePts.m_stPts.empty();
    m_mtxMapFeatures.unlock();
}

/**@brief timer callback function to predict particles;
 *
 * timer callback function to predict particles;
 * @param event [IN]: not used;
 */
void Localizer::CallbackPredictTimer(const ros::TimerEvent &event)
{
    if (!m_bParticlesInitialized)
    {
        return;
    }


    static double last_timestamp = ros::Time::now().toSec();
    auto current_timestamp = ros::Time::now().toSec();

    double wheel_velocity, yaw_d;
    //get current velocity;
    m_mtxWheelSpeed.lock_shared();
    wheel_velocity = m_fWheelSpeed;
    m_mtxWheelSpeed.unlock_shared();

    //get current yaw_d;
    m_mtxYawVelocity.lock_shared();
    yaw_d = m_fYawVelocity;
    m_mtxYawVelocity.unlock_shared();

    m_stEKF.Predict(current_timestamp, wheel_velocity, yaw_d);

    Eigen::Matrix<double, 5, 1> pose;
    m_stEKF.GetPose(pose);

    double ux, uy, uz;

    TransformEngineerToUTM(pose(0), pose(1), 0.0, ux, uy, uz);

    Vector4d prior_pose;
    prior_pose << ux, uy, uz, pose(2);

    PublishPriorPose(ros::Time::now(), prior_pose);
    PublishPriorTrajectory(ros::Time::now(), pose);

    last_timestamp = current_timestamp;

}

/**@brief transform wgs84 gps point into engineer frame;
 *
 * transform wgs84 gps point into engineer frame;
 * 1. transform wgs84 gps point into ecef frame;
 * 2. trnasform ecef frame into engineer frame;
 * @param latitude [IN]: latitude of the gps point;
 * @param longitude [IN]: longitude of the gps point;
 * @param altitude [IN]: altitude of the gps point;
 * @param ex [OUT]: x in engineer frame;
 * @param ey [OUT]: y in engineer frame;
 * @param ez [OUT]: z in engineer frame;
 */
void Localizer::TransformWGS84ToEngineer(double latitude, double longitude, double altitude, double &ex, double &ey, double &ez)
{
    //transform wgs84 to ecef;
    auto wx = longitude * DEGREE_TO_RADIAN;
    auto wy = latitude * DEGREE_TO_RADIAN;
    auto wz = altitude;
    auto ret = pj_transform(m_stProjWGS84, m_stProjECEF, 1, 1, &wx, &wy, &wz);
    LOG_IF(FATAL, ret != 0) << "proj cannot transform gps point, latitude:" << latitude << ", longitude:" << longitude << ", altitude:" << altitude;

    //transform ecef to engineer;
    double tx = 0.0, ty = 0.0, tz = 0.0, tw = 0.0;
    tx = wx * m_emECEF2Eng(0, 0) + wy * m_emECEF2Eng(1, 0) + wz * m_emECEF2Eng(2, 0) + m_emECEF2Eng(3, 0);
    ty = wx * m_emECEF2Eng(0, 1) + wy * m_emECEF2Eng(1, 1) + wz * m_emECEF2Eng(2, 1) + m_emECEF2Eng(3, 1);
    tz = wx * m_emECEF2Eng(0, 2) + wy * m_emECEF2Eng(1, 2) + wz * m_emECEF2Eng(2, 2) + m_emECEF2Eng(3, 2);
    tw = wx * m_emECEF2Eng(0, 3) + wy * m_emECEF2Eng(1, 3) + wz * m_emECEF2Eng(2, 3) + m_emECEF2Eng(3, 3);
    ex = tx / tw;
    ey = ty / tw;
    ez = tz / tw;
}

/**@brief transform engineer coordinate into WGS84 coordiante;
 *
 * transform engineer coordinate into WGS84 coordinate;
 * @param ex [IN]: x in engineer frame;
 * @param ey [IN]: y in engineer frame;
 * @param ez [IN]: z in engineer frame;
 * @param latitude [OUT]: latitude, in deg;
 * @param longitude [OUT]: latitude, in deg;
 * @param altitude [OUT]: altitude, in meter;
 */
void Localizer::TransformEngineerToWGS84(double ex, double ey, double ez, double &latitude, double &longitude, double &altitude)
{
    Eigen::Matrix<double, 1, 4> eng;
    eng << ex, ey, ez, 1.0;
    auto ecef = eng * m_emEng2ECEF;

    longitude = ecef(0);
    latitude = ecef(1);
    altitude = ecef(2);

    auto ret = pj_transform(m_stProjECEF, m_stProjWGS84, 1, 1, &longitude, &latitude, &altitude);
    LOG_IF(FATAL, ret != 0) << "proj cannot transform engineer point:" << ex << ", " << ey << "," << ez;
    latitude *= RADIAN_TO_DEGREE;
    longitude *= RADIAN_TO_DEGREE;
}

/**@brief transform engineer to utm;
 *
 * transform engineer to utm;
 * @param ex [IN]: x coordinate in engineer;
 * @param ey [IN]: y coordinate in engineer;
 * @param ez [IN]: z coordinate in engineer;
 * @param ux [OUT]: x coordinate in utm;
 * @param uy [OUT]: y coordinate in utm;
 * @param uz [OUT]: z coordinate in utm;
 */
void Localizer::TransformEngineerToUTM(double ex, double ey, double ez, double &ux, double &uy, double &uz)
{
    double latitude, longitude, altitude;
    TransformEngineerToWGS84(ex, ey, ez, latitude, longitude, altitude);

    m_mtxGnssInfo.lock_shared();
    altitude = m_stGnssInfo.fAltitude;
    m_mtxGnssInfo.unlock_shared();

    auto pt = CWgs84Utils::Wgs84ToUtm(longitude, latitude, altitude);
    int zone = floor((longitude + 180.0) / 6) + 1;
    pt.x -= zone * 1000000;
    ux = pt.x;
    uy = pt.y;
    uz = pt.z;
}

/**@brief load transform from string;
 *
 * load ECEF to enigneer frame from transform string;
 * @param str [IN]: the transform string, specified the string by gflag parameters;
 * @return always return true;
 */
bool Localizer::LoadTransform(const string &str)
{
    double transform[4][4];

    vector<string> strs;
    boost::algorithm::split(strs, str, boost::is_any_of(","));
    LOG_IF(FATAL, strs.size() != 16) << "invalid transform string" << str << ", size:" << strs.size();

    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            transform[r][c] = atof(strs[r * 4 + c].data());
        }
    }

    Matrix4d mat;
    mat << transform[0][0], transform[0][1], transform[0][2], transform[0][3], transform[1][0], transform[1][1], transform[1][2], transform[1][3], transform[2][0], transform[2][1], transform[2][2], transform[2][3], transform[3][0], transform[3][1], transform[3][2], transform[3][3];
    m_emECEF2Eng = mat.inverse();
    m_emEng2ECEF = mat;
    return true;
}

/**@brief callback function to process gnss heading;
 *
 * callback function to process gnss heading, as the heading may not available, use gnss velocity to calculate the heading instead;
 * @param msg [IN]: the gnss heading message;
 * @note do not use this function;
 */
void Localizer::CallbackGnssHeading(const geometry_msgs::TwistStampedConstPtr &msg)
{
    m_mtxGnssInfo.lock();
    m_mtxGnssInfo.unlock();
}

/**@brief callback function to process gnss position;
 *
 * callback function to process gnss position:
 * 1. update gnss info;
 * 2. if the particles is not initialized, try to initialize the particles;
 * 3. broadcast transform between world->engineer->vehicle->rslidar;
 * @param msg [IN]: the gnss position;
 */
void Localizer::CallbackGnssPosition(const sensor_msgs::NavSatFixConstPtr &msg)
{
    if (isnan(msg->latitude) || isnan(msg->longitude) || isnan(msg->altitude))
    {
        char timestamp[128];
        sprintf(timestamp, "%.6lf", msg->header.stamp.toSec());
        LOG(WARNING) << "gnss not a number, at:" << timestamp;
        return;
    }

    double mars_lat, mars_lon;
    CWgs84Utils::Wgs84ToMars(msg->latitude, msg->longitude, mars_lat, mars_lon);

    static double last_time_stamp = msg->header.stamp.toSec();
    if (msg->header.stamp.toSec() < last_time_stamp)
    {
        m_bParticlesInitialized = false;
        m_stGnssInfo.bHeadingInitialized = false;
        m_stGnssInfo.bPositionInitialized = false;
        m_stGnssInfo.bVelocityInitialized = false;
    }
    last_time_stamp = msg->header.stamp.toSec();


    double ex = 0.0, ey = 0.0, ez = 0.0;
    TransformWGS84ToEngineer(mars_lat, mars_lon, msg->altitude, ex, ey, ez);

    m_mtxGnssInfo.lock();
    m_stGnssInfo.fTimeStamp = msg->header.stamp.toSec();
    m_stGnssInfo.fLatitude = mars_lat;
    m_stGnssInfo.fLongitude = mars_lon;
    m_stGnssInfo.fAltitude = msg->altitude;
    m_stGnssInfo.fEngX = ex;
    m_stGnssInfo.fEngY = ey;
    m_stGnssInfo.bPositionInitialized = true;

    if (!m_bParticlesInitialized && m_stGnssInfo.bPositionInitialized && m_stGnssInfo.bHeadingInitialized)
    {
        InitializeParticles(m_stGnssInfo.fEngX, m_stGnssInfo.fEngY, m_stGnssInfo.fHeading);
    }
    m_mtxGnssInfo.unlock();

    if (FLAGS_debug)
    {
        static tf::TransformBroadcaster tf_broadcaster;

        tf::Transform transform;
        transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        tf_broadcaster.sendTransform(tf::StampedTransform{transform, msg->header.stamp, "/world", "/engineer"});
        tf_broadcaster.sendTransform(tf::StampedTransform{transform, msg->header.stamp, "/world", "/map"});
        transform.setOrigin(tf::Vector3(ex, ey, 0.0));
        m_mtxGnssInfo.lock_shared();
        transform.setRotation(m_stGnssInfo.stQuat);
        m_mtxGnssInfo.unlock_shared();

        tf_broadcaster.sendTransform(tf::StampedTransform{transform, msg->header.stamp, "/engineer", "/vehicle"});
        transform.setOrigin(tf::Vector3(0.72, 0.0, 1.3));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
        tf_broadcaster.sendTransform(tf::StampedTransform{transform, msg->header.stamp, "/vehicle", "/rslidar"});
    }

    PublishGpsTrajectory(msg->header.stamp, ex, ey);
}

/**@brief calculate heading;
 *
 * calculate heading by velocity east and velocity north;
 * @param vx [IN]: velocity east;
 * @param vy [IN]: velocity north;
 * @param heading [OUT]: heading angle, in radian;
 */
void Localizer::CalcHeading(double vx, double vy, double &heading)
{
    heading = acos(vx / sqrt(vx * vx + vy * vy + 1e-4));
    heading = vy >= 0.0 ? heading : PI_DOUBLE - heading;
}

/**@brief callback function to process gnss velocity;
 *
 * callback function to process gnss velocity, heading angle is caluclated by velocity;
 * @param msg [IN]: the gnss velocity message;
 */
void Localizer::CallbackGnssVelocity(const geometry_msgs::TwistStampedConstPtr &msg)
{
    if (isnan(msg->twist.linear.x) || isnan(msg->twist.linear.y))
    {
        char timestamp[128];
        sprintf(timestamp, "%.6lf", msg->header.stamp.toSec());
        LOG(WARNING) << "gnss velocity not a number, at " << timestamp;
        return;
    }

    double heading = 0.0;
    CalcHeading(msg->twist.linear.x, msg->twist.linear.y, heading);
    m_mtxGnssInfo.lock();
    m_stGnssInfo.fVelEast = msg->twist.linear.x;
    m_stGnssInfo.fVelNorth = msg->twist.linear.y;
    m_stGnssInfo.fHeading = heading;
    m_stGnssInfo.stQuat.setEuler(0.0, 0.0, heading);
    m_stGnssInfo.bVelocityInitialized = true;
    m_stGnssInfo.bHeadingInitialized = true;
    m_mtxGnssInfo.unlock();
}

/**@brief callback function to process imu;
 *
 * callback function to process imu, simply update yaw velocity;
 * @param msg [IN]: the imu message;
 */
void Localizer::CallbackImu(const sensor_msgs::ImuConstPtr &msg)
{
    tf::Quaternion quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    double yaw, pitch, roll;
    Quaternion2Euler(quaternion, yaw, pitch, roll);

    //    LOG_IF(ERROR, fabs(msg->angular_velocity.z)>0.1)<<"raw imu yaw velocity:"<<msg->angular_velocity.z*RADIAN_TO_DEGREE;
    static deque<double> yaw_vels;
    yaw_vels.push_back(msg->angular_velocity.z);
    if (yaw_vels.size() > 100)
    {
        yaw_vels.pop_front();
    }

    double sum = 0.0;
    for (auto yaw_vel : yaw_vels)
    {
        sum += yaw_vel;
    }
    double avg_yaw_vel = sum / yaw_vels.size();

    m_mtxYawVelocity.lock();
    m_fYawVelocity = avg_yaw_vel;
    m_mtxYawVelocity.unlock();
}

/**@brief transform quaternion to euler angles;
 *
 * transform quaternion to euler angles;
 * @param quat [IN]: the quaternion;
 * @param yaw [OUT]: the yaw angle, in radian;
 * @param pitch [OUT]: the pitch angle, in radian;
 * @param roll [OUT]: the roll angle, in radian;
 */
void Localizer::Quaternion2Euler(const tf::Quaternion &quat, double &yaw, double &pitch, double &roll)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
    double cosr = +1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
    if (fabs(sinp) >= 1)
    {
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
        pitch = asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny = +2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
    double cosy = +1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
    yaw = atan2(siny, cosy);
}

/**@brief sample particles around gps;
 *
 * sample particles around gps, not used because of the frequency;
 * @param gps_position [IN]: the gps position;
 */
void Localizer::SampleGpsParticles()
{
    m_mtxGnssInfo.lock_shared();
    double timestamp = m_stGnssInfo.fTimeStamp;
    double eng_x = m_stGnssInfo.fEngX;
    double eng_y = m_stGnssInfo.fEngY;
    double vel_x = m_stGnssInfo.fVelEast;
    double vel_y = m_stGnssInfo.fVelNorth;
    double heading = m_stGnssInfo.fHeading;
    m_mtxGnssInfo.unlock_shared();

    auto delta_time = 0.0;//ros::Time::now().toSec() - timestamp;
    //    fprintf(stderr, "delta time:%.6lf\n", delta_time);
    static std::default_random_engine random_engine;
    std::normal_distribution<double> dist_x(eng_x + vel_x * delta_time, 0.5);
    std::normal_distribution<double> dist_y(eng_y + vel_y * delta_time, 0.5);
    std::normal_distribution<double> dist_heading(heading, 0.01);

    auto weight = 1.0 / (FLAGS_num_imu + FLAGS_num_gps);
    for (int p = 0; p < FLAGS_num_gps; ++p)
    {
        auto yaw = dist_heading(random_engine);
        auto pos_x = dist_x(random_engine);
        auto pos_y = dist_y(random_engine);
        NormalizeYaw(yaw);
        m_vecParticles.emplace_back(Particle(p + FLAGS_num_imu, pos_x, pos_y, yaw, weight));
    }
}

/**@brief initialize particles based on gnss position and heading;
 *
 * initialize particles based on gnss position and heading;
 * @param gps_x [IN]: the gps position x;
 * @param gps_y [IN]: the gps position y;
 * @param heading [IN]: the gps heading;
 */
void Localizer::InitializeParticles(double gps_x, double gps_y, double heading)
{
    m_mtxParticles.lock();
    std::default_random_engine random_engine;
    std::normal_distribution<double> dist_x(gps_x, m_fGpsStdVar);
    std::normal_distribution<double> dist_y(gps_y, m_fGpsStdVar);
    std::normal_distribution<double> dist_heading(heading, m_fHeadingStdVar);

    m_vecParticles.clear();
    m_vecParticles.reserve(FLAGS_num_gps + FLAGS_num_imu);

    auto weight = 1.0 / (FLAGS_num_gps + FLAGS_num_imu);
    for (int p = 0; p < FLAGS_num_imu + FLAGS_num_gps; ++p)
    {
        m_vecParticles.emplace_back(Particle{p, dist_x(random_engine), dist_y(random_engine), dist_heading(random_engine), weight});
    }

    m_bParticlesInitialized = true;
    m_mtxParticles.unlock();

    m_mtxWheelSpeed.lock_shared();
    double velocity = m_fWheelSpeed;
    m_mtxWheelSpeed.unlock_shared();

    m_mtxYawVelocity.lock_shared();
    double yaw_rate = m_fYawVelocity;
    m_mtxYawVelocity.unlock_shared();
    m_stEKF.Initialize(ros::Time::now().toSec(), gps_x, gps_y, heading, velocity, yaw_rate);
}

/**@brief resample particles;
 *
 * resample particles according to particle weight;
 * @todo add resample gps particles;
 */
void Localizer::ResampleParticles()
{
    static std::default_random_engine random_engine;
    std::vector<double> weights;
    vector<Particle> particles_copy;
    for (int p = 0; p < m_vecParticles.size(); ++p)
    {
        particles_copy.emplace_back(m_vecParticles[p]);
        weights.emplace_back(m_vecParticles[p].m_fWeight);
    }

    std::discrete_distribution<int> dist(weights.begin(), weights.end());
    std::normal_distribution<double> dist_x(0.0, 0.1);
    std::normal_distribution<double> dist_y(0.0, 0.1);
    std::normal_distribution<double> dist_heading(0.0, 0.01);

    m_vecParticles.clear();
    for (int p = 0; p < FLAGS_num_imu; ++p)
    {
        auto particle = particles_copy[dist(random_engine)];
        particle.m_evPose(0) += dist_x(random_engine);
        particle.m_evPose(1) += dist_y(random_engine);
        particle.m_evPose(2) += dist_heading(random_engine);

        NormalizeYaw(particle.m_evPose(2));
        m_vecParticles.emplace_back(particle);
    }

    SampleGpsParticles();
}

/**@brief get transform between source points to target points by ICP;
 *
 * get transform between source points to target points by ICP based on svd;
 * @param source_pts [IN]: source points;
 * @param target_pts [IN]: target points;
 * @param threshold [IN]: threshold of the distance between transformed source points to target points;
 * @param tf [IN|OUT]: the initial and the result transform;
 * @note
 */
void Localizer::ICP(const PointsVector<PointXYZ> &source_pts, const PointsVector<PointXYZ> &target_pts, double threshold, tf::Transform &tf)
{
    auto vector = tf.getOrigin();
    auto num_of_pts = source_pts.size();

    PointXYZ mean_source;
    source_pts.GetMeanPoint(mean_source);
    mean_source.z() = 0.0;

    PointXYZ mean_target;
    target_pts.GetMeanPoint(mean_target);
    mean_target.z() = 0.0;

    Eigen::MatrixXd mat_X, mat_Y, mat_W;
    mat_X.resize(3, source_pts.size());
    mat_Y.resize(3, source_pts.size());
    mat_W.resize(source_pts.size(), source_pts.size());

    mat_X.setZero();
    mat_Y.setZero();
    mat_W.setZero();

    PointXYZ delta_pt;
    double square_dist = 0.0;

    double sum_square_dist = 0.0;
    for (int p = 0; p < num_of_pts; ++p)
    {
        auto tf_source_pt = tf * tf::Vector3{source_pts[p].x(), source_pts[p].y(), source_pts[p].z()};
        delta_pt = PointXYZ{tf_source_pt.x(), tf_source_pt.y(), tf_source_pt.z()} - target_pts[p];
        delta_pt.z() = 0.0;
        square_dist = delta_pt.x() * delta_pt.x() + delta_pt.y() * delta_pt.y() + delta_pt.z() * delta_pt.z();

        sum_square_dist += square_dist;
        if (square_dist > threshold * threshold)
        {
            mat_W(p, p) = 0.0;
            continue;
        }

        delta_pt = source_pts[p] - mean_source;
        delta_pt.z() = 0.0;
        mat_X(0, p) = delta_pt.x();
        mat_X(1, p) = delta_pt.y();
        mat_X(2, p) = delta_pt.z();

        delta_pt = target_pts[p] - mean_target;
        delta_pt.z() = 0.0;
        mat_Y(0, p) = delta_pt.x();
        mat_Y(1, p) = delta_pt.y();
        mat_Y(2, p) = delta_pt.z();

        mat_W(p, p) = (40.0 - sqrt(source_pts[p].x() * source_pts[p].x() + source_pts[p].y() * source_pts[p].y())) / 40.0;
    }

    //calculate S
    auto mat_S = mat_X * mat_W * mat_Y.transpose();

    //calculate U,V
    Eigen::JacobiSVD<Eigen::Matrix<double, 3, 3> > svd(mat_S, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    //calculate R;
    Eigen::Matrix3d R = V * U.transpose();
    Eigen::Quaternion<double> quaternion(R);

    double yaw, pitch, roll;
    Quaternion2Euler(tf::Quaternion{quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()}, yaw, pitch, roll);

    tf.setRotation(tf::Quaternion{quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()});

    //calculate t;
    auto tf_pt = quaternion * Vector3d{mean_source.x(), mean_source.y(), mean_source.z()};
    delta_pt = mean_target - PointXYZ{tf_pt.x(), tf_pt.y(), tf_pt.z()};

    tf.setOrigin(tf::Vector3{delta_pt.x(), delta_pt.y(), 0.0});
}

/**@brief match the feature points to map points by icp;
 *
 * match the feature points to the map points by ICP;
 * @param marker_pts [IN]: marker points extracted from lidar scan in lidar frame;
 * @param edge_pts [IN]: edge points extract from lidar scan in lidar frame;
 * @param pole_pts [IN]: pole points extract from lidar scan in lidar frame;
 * @param init_pose [IN]: initial pose of the vehicle relative to engineer frame;
 * @param result_pose [OUT]: matched pose of the vehicle relative to engineer frame;
 * @note the localization based on icp is not stable;
 */
void Localizer::ICPMatch(const PointsVector<PointXYZ> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, const Vector3d &init_pose, Vector3d &result_pose)
{
    //transform the points into vehicle frame;
    tf::Transform tf_lidar_vehicle;
    tf_lidar_vehicle.setOrigin(tf::Vector3(0.72, 0.0, 1.3));
    tf_lidar_vehicle.setRotation(tf::Quaternion{0.0, 0.0, 0.0, 1.0});

    PointsVector<PointXYZ> pole_pts_veh;//pole points in vehicle frame;
    TransformPts(tf_lidar_vehicle, pole_pts, pole_pts_veh);

    PointsVector<PointXYZ> tf_pole_pts;
    PointsVector<PointXYZ> nearest_pole_pts;

    result_pose = init_pose;
    tf::Transform tf_veh_eng;
    //initialize transform
    tf_veh_eng.setOrigin(tf::Vector3{result_pose(0), result_pose(1), 0.0});
    //    tf_veh_eng.setRotation(tf::Quaternion{0.0, 0.0, result_pose(2)});
    tf_veh_eng.setRotation(tf::createQuaternionFromYaw(result_pose(2)));

    double threshold = 4.0;
    //iterative ICP
    for (int i = 0; i < 20; ++i)
    {
        tf_pole_pts.clear();

        //transform the points;
        TransformPts(tf_veh_eng, pole_pts_veh, tf_pole_pts);

        for (auto &pt:tf_pole_pts)
        {
            pt.z() = 0.0;
        }

        //find nearest features;
        m_stMapPolePts.GetFLANNNearestPts(tf_pole_pts, nearest_pole_pts);

        //icp by svd;
        ICP(pole_pts_veh, nearest_pole_pts, threshold, tf_veh_eng);

        threshold = max(0.5, threshold * 0.8);
    }
    TransformPts(tf_veh_eng, pole_pts_veh, tf_pole_pts);

    //    //publish localized pole points to visualize the icp match result;
    //    m_msgLocalizedPolePts.header.frame_id = "/engineer";
    //    m_msgLocalizedPolePts.points.clear();
    //    m_msgLocalizedPolePts.points.reserve(pole_pts.size());
    //    for (int p = 0; p < pole_pts.size(); ++p)
    //    {
    //        geometry_msgs::Point32 pt;
    //        pt.x = tf_pole_pts[p].x();
    //        pt.y = tf_pole_pts[p].y();
    //        pt.z = tf_pole_pts[p].z();
    //        m_msgLocalizedPolePts.points.emplace_back(pt);
    //    }
    //    m_pubLocalizedPolePts.publish(m_msgLocalizedPolePts);
}

/**@brief callback function to process wheel speed;
 *
 * callback function to process wheel speed, simply update the wheel speed;
 * @param msg [IN]: the wheel speed message;
 */
void Localizer::CallbackWheelSpeed(const nav_msgs::OdometryConstPtr &msg)
{
    m_mtxWheelSpeed.lock();
    m_fWheelSpeed = msg->twist.twist.linear.x;
    m_mtxWheelSpeed.unlock();
}

/**@brief update particles based on feature points measurements;
 *
 * update particles based on feature points measurement:
 * 1. evaluate particles based on new feature points;
 * 2. normalize particle weight;
 * 3. resample particles;
 * @param marker_pts [IN]: marker points;
 * @param edge_pts [IN]: edge points;
 * @param pole_pts [IN]: pole points;
 * @param best_pose [OUT]: best localized pose;
 */
void Localizer::UpdateParticles(const ros::Time &timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, Vector3d &best_pose)
{
    m_mtxParticles.lock();

    static auto last_timestamp = timestamp.toSec();

    m_mtxWheelSpeed.lock_shared();
    auto wheel_velocity = m_fWheelSpeed;
    m_mtxWheelSpeed.unlock_shared();

    m_mtxYawVelocity.lock_shared();
    auto yaw_vel = m_fYawVelocity;
    m_mtxYawVelocity.unlock_shared();

    //predict particles;
    for (auto &particle: m_vecParticles)
    {
        particle.CTRVPrediction(timestamp.toSec() - last_timestamp, wheel_velocity, yaw_vel, m_fWheelSpeedStdVar, m_fYawVeloStdVar);
    }
    last_timestamp = timestamp.toSec();

    m_mtxMapFeatures.lock();
    EvaluateParticles(marker_pts, edge_pts, pole_pts);
    m_mtxMapFeatures.unlock();
    double max_weight = 0.0;
    double confidence = 0.0;
    for (int i = 0; i < FLAGS_num_gps + FLAGS_num_imu; ++i)
    {
        if (m_vecParticles[i].m_fWeight > max_weight)
        {
            max_weight = m_vecParticles[i].m_fWeight;
            best_pose = m_vecParticles[i].m_evPose;
            confidence = m_vecParticles[i].m_fInlierRatio;
        }
    }

    LOG(INFO) << "update particles, max weight:" << max_weight << ", confidence:" << confidence << ", pose:" << best_pose(0) << ", " << best_pose(1) << ", " << best_pose(2) * RADIAN_TO_DEGREE;

    for (int i = 0; i < FLAGS_num_gps + FLAGS_num_imu; ++i)
    {
        m_vecParticles[i].m_fWeight /= max_weight;
    }

    //resample particles;
    ResampleParticles();
    m_mtxParticles.unlock();

    m_stEKF.Update(timestamp.toSec(), best_pose(0), best_pose(1), best_pose(2), wheel_velocity, yaw_vel, confidence);
}

/**@brief evalulate particles by new feature points measurement;
 *
 * evaluate particles by new feature points measurement:
 * 1. transform feature points according to particle pose;
 * 2. find nearest feature points;
 * 3. update particle weight;
 * @param marker_pts [IN]: marker points;
 * @param edge_pts [IN]: edge points;
 * @param pole_pts [IN]: pole points;
 * @note if the time is strictly synced, predict each particle before transforming feature points;
 */
void Localizer::EvaluateParticles(const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts)
{
    auto num_of_particles = FLAGS_num_imu + FLAGS_num_gps;
    PointsVector<PointXYZ> tf_marker_pts, tf_pole_pts, tf_edge_pts;

    //    PointsWriter::WritePts2Asc(marker_pts, GetCurrentDateTimeMilliSecondString()+std::string("_markers.asc"));
    //    PointsWriter::WritePts2Asc(edge_pts, GetCurrentDateTimeMilliSecondString()+std::string("_edges.asc"));
    //    PointsWriter::WritePts2Asc(pole_pts, GetCurrentDateTimeMilliSecondString()+std::string("_poles.asc"));
    if (m_bUseMarkerPts && FLAGS_use_markers)
    {
        tf_marker_pts.resize(num_of_particles * marker_pts.size());
    }
    if (m_bUsePolePts)
    {
        tf_pole_pts.resize(num_of_particles * pole_pts.size());
    }

    if (m_bUseEdgePts)
    {
        tf_edge_pts.resize(num_of_particles * edge_pts.size());
    }

    //copy transformed points;
    //#pragma omp parallel for
    for (int p = 0; p < num_of_particles; ++p)
    {
        auto cos_yaw = cos(m_vecParticles[p].m_evPose(2));
        auto sin_yaw = sin(m_vecParticles[p].m_evPose(2));

        if (m_bUsePolePts)
        {
            for (int i = 0; i < pole_pts.size(); i++)
            {
                auto idx = p * pole_pts.size() + i;
                tf_pole_pts[idx].x() = cos_yaw * (pole_pts[i].x() + 1.3) - sin_yaw * pole_pts[i].y() + m_vecParticles[p].m_evPose(0);
                tf_pole_pts[idx].y() = sin_yaw * (pole_pts[i].x() + 1.3) + cos_yaw * pole_pts[i].y() + m_vecParticles[p].m_evPose(1);
                tf_pole_pts[idx].z() = 0.0;
            }
        }

        if (m_bUseEdgePts)
        {
            for (int i = 0; i < edge_pts.size(); i++)
            {
                auto idx = p * edge_pts.size() + i;
                tf_edge_pts[idx].x() = cos_yaw * (edge_pts[i].x() + 1.3) - sin_yaw * edge_pts[i].y() + m_vecParticles[p].m_evPose(0);
                tf_edge_pts[idx].y() = sin_yaw * (edge_pts[i].x() + 1.3) + cos_yaw * edge_pts[i].y() + m_vecParticles[p].m_evPose(1);
                tf_edge_pts[idx].z() = 0.0;
            }
        }

        if (m_bUseMarkerPts && FLAGS_use_markers)
        {
            for (int i = 0; i < marker_pts.size(); i++)
            {
                auto idx = p * marker_pts.size() + i;
                tf_marker_pts[idx].x() = cos_yaw * (marker_pts[i].x() + 1.3) - sin_yaw * marker_pts[i].y() + m_vecParticles[p].m_evPose(0);
                tf_marker_pts[idx].y() = sin_yaw * (marker_pts[i].x() + 1.3) + cos_yaw * marker_pts[i].y() + m_vecParticles[p].m_evPose(1);
                tf_marker_pts[idx].z() = 0.0;
            }
        }
    }


    //get nearest points;
    PointsVector<PointXYZ> nearest_marker_pts, nearest_pole_pts, nearest_edge_pts;
    if (m_bUseMarkerPts && FLAGS_use_markers)
    {
        //        PointsWriter::WritePts2Asc(tf_marker_pts, GetCurrentDateTimeString()+std::string(".asc"));
        m_stMapMarkerPts.GetFLANNNearestPts(tf_marker_pts, nearest_marker_pts);
    }


    if (m_bUseEdgePts)
    {
        m_stMapEdgePts.GetFLANNNearestPts(tf_edge_pts, nearest_edge_pts);
    }

    if (m_bUsePolePts)
    {
        m_stMapPolePts.GetFLANNNearestPts(tf_pole_pts, nearest_pole_pts);
    }


    const double MAP_STD_VAR = 0.4;

    //#pragma omp parallel for
    for (int p = 0; p < num_of_particles; p++)
    {
        auto dist_sum = 0.0;
        int inlier_count = 0;
        if (m_bUsePolePts)
        {
            for (int i = 0; i < pole_pts.size(); ++i)
            {
                auto idx = p * pole_pts.size() + i;
                auto delta_x = fabs(nearest_pole_pts[idx].x() - tf_pole_pts[idx].x());
                auto delta_y = fabs(nearest_pole_pts[idx].y() - tf_pole_pts[idx].y());

                if (delta_x > MAP_STD_VAR || delta_y > MAP_STD_VAR)
                {
                    dist_sum += 9 * MAP_STD_VAR * MAP_STD_VAR;
                }
                else
                {
                    dist_sum += delta_x * delta_x + delta_y * delta_y;
                    inlier_count++;
                }
            }
        }

        if (m_bUseEdgePts)
        {
            for (int i = 0; i < edge_pts.size(); ++i)
            {
                auto idx = p * edge_pts.size() + i;
                auto delta_x = fabs(nearest_edge_pts[idx].x() - tf_edge_pts[idx].x());
                auto delta_y = fabs(nearest_edge_pts[idx].y() - tf_edge_pts[idx].y());


                if (delta_x > MAP_STD_VAR || delta_y > MAP_STD_VAR)
                {
                    dist_sum += 9 * MAP_STD_VAR * MAP_STD_VAR;
                }
                else
                {
                    dist_sum += delta_x * delta_x + delta_y * delta_y;
                    inlier_count++;
                }
            }
        }

        if (m_bUseMarkerPts && FLAGS_use_markers)
        {
            for (int i = 0; i < marker_pts.size(); ++i)
            {
                auto idx = p * marker_pts.size() + i;
                auto delta_x = fabs(nearest_marker_pts[idx].x() - tf_marker_pts[idx].x());
                auto delta_y = fabs(nearest_marker_pts[idx].y() - tf_marker_pts[idx].y());

                if (delta_x > MAP_STD_VAR || delta_y > MAP_STD_VAR)
                {
                    dist_sum += 9 * MAP_STD_VAR * MAP_STD_VAR;
                }
                else
                {
                    dist_sum += delta_x * delta_x + delta_y * delta_y;
                    inlier_count++;
                }
            }
        }

        auto num_of_features = ((m_bUseMarkerPts && FLAGS_use_markers) ? marker_pts.size() : 0) + (m_bUseEdgePts ? edge_pts.size() : 0) + (m_bUsePolePts ? pole_pts.size() : 0);
        m_vecParticles[p].m_fWeight = exp(-.5 * dist_sum / (MAP_STD_VAR * MAP_STD_VAR * num_of_features));
        m_vecParticles[p].m_fInlierRatio = inlier_count * 1.0 / (num_of_features);
    }
}

/**@brief evaluate particle based on new feature points;
 *
 * evaluate particle based on new feature points:
 * 1. transform feature points according to particle pose;
 * 2. get nearest feature points in the map;
 * 3. update the particle weight;
 * @param marker_pts [IN]: marker points;
 * @param edge_pts [IN]: edge points;
 * @param pole_pts [IN]: pole points;
 * @param particle [IN|OUT]: the particle to be evaluated;
 */
void Localizer::EvaluateParticle(const PointsVector<PointXYZ> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, Particle &particle)
{
    PointsVector<PointXYZ> tf_edge_pts, tf_pole_pts;

    auto cos_yaw = cos(particle.m_evPose(2));
    auto sin_yaw = sin(particle.m_evPose(2));

    //transform points;
    tf_edge_pts.resize(edge_pts.size());
    for (const auto &pt:edge_pts)
    {
        auto px = cos_yaw * (pt.x() + 1.3) - sin_yaw * pt.y() + particle.m_evPose(0);
        auto py = sin_yaw * (pt.x() + 1.3) + cos_yaw * pt.y() + particle.m_evPose(1);

        tf_edge_pts.emplace_back(PointXYZ{px, py, 0.0});
    }

    tf_pole_pts.resize(pole_pts.size());
    for (const auto &pt:pole_pts)
    {
        auto px = cos_yaw * (pt.x() + 1.3) - sin_yaw * pt.y() + particle.m_evPose(0);
        auto py = sin_yaw * (pt.x() + 1.3) + cos_yaw * pt.y() + particle.m_evPose(1);

        tf_pole_pts.emplace_back(PointXYZ{px, py, 0.0});
    }

    PointsVector<PointXYZ> nearest_edge_pts, nearest_pole_pts;
    //    m_stMapEdgePts.GetFLANNNearestPts(tf_edge_pts, nearest_edge_pts);
    m_stMapPolePts.GetFLANNNearestPts(tf_pole_pts, nearest_pole_pts);

    double dist_sum = 0.0;
    const double MAP_STD_VAR = 0.5;
    for (int i = 0; i < nearest_pole_pts.size(); ++i)
    {
        auto delta_x = fabs(nearest_pole_pts[i].x() - tf_pole_pts[i].x());
        auto delta_y = fabs(nearest_pole_pts[i].y() - tf_pole_pts[i].y());

        dist_sum += (delta_x > MAP_STD_VAR || delta_y > MAP_STD_VAR) ? 9 * MAP_STD_VAR * MAP_STD_VAR : delta_x * delta_x + delta_y * delta_y;
    }

    //    for (int i = 0; i < nearest_edge_pts.size(); ++i)
    //    {
    //        auto delta_x = fabs(nearest_edge_pts[i].x() - tf_edge_pts[i].x());
    //        auto delta_y = fabs(nearest_edge_pts[i].y() - tf_edge_pts[i].y());
    //
    //        dist_sum += (delta_x > MAP_STD_VAR || delta_y > MAP_STD_VAR) ? 9 * MAP_STD_VAR * MAP_STD_VAR : delta_x * delta_x + delta_y * delta_y;
    //    }

    particle.m_fWeight = exp(-.5 * dist_sum / (MAP_STD_VAR * MAP_STD_VAR * (pole_pts.size())));
}

/**@brief callback function to update local map edge points;
 *
 * @param msg [IN]: the edge points in local map;
 */
void Localizer::CallbackMapEdgePts(const sensor_msgs::PointCloudConstPtr &msg)
{
    m_mtxMapFeatures.lock();
    m_stMapEdgePts.m_stPts.clear();
    m_stMapEdgePts.m_stPts.reserve(msg->points.size());
    for (const auto &pt:msg->points)
    {
        m_stMapEdgePts.m_stPts.emplace_back(PointXYZ{pt.x, pt.y, pt.z});
    }

    if (m_stMapEdgePts.m_stPts.empty())
    {
        m_bUseEdgePts = false;
    }
    else
    {
        m_bUseEdgePts = true;
        m_stMapEdgePts.BuildFLANNIndex();
    }
    m_mtxMapFeatures.unlock();
}

/**@brief callback function to update local map marker points;
 *
 * @param msg [IN]: the marker points in local map;
 */
void Localizer::CallbackMapMarkerPts(const sensor_msgs::PointCloudConstPtr &msg)
{
    m_mtxMapFeatures.lock();
    m_stMapMarkerPts.m_stPts.clear();
    m_stMapMarkerPts.m_stPts.reserve(msg->points.size());
    for (const auto &pt:msg->points)
    {
        m_stMapMarkerPts.m_stPts.emplace_back(PointXYZ{pt.x, pt.y, pt.z});
    }

    if (m_stMapMarkerPts.m_stPts.empty())
    {
        m_bUseMarkerPts = false;
    }
    else
    {
        m_bUseMarkerPts = true;
        m_stMapMarkerPts.BuildFLANNIndex();
    }
    m_mtxMapFeatures.unlock();
}

/**@brief callback function to update local map pole points;
 *
 * @param msg [IN]: the pole points in local map;
 */
void Localizer::CallbackMapPolePts(const sensor_msgs::PointCloudConstPtr &msg)
{
    m_mtxMapFeatures.lock();
    m_stMapPolePts.m_stPts.clear();
    m_stMapPolePts.m_stPts.reserve(msg->points.size());

    for (const auto &pt:msg->points)
    {
        m_stMapPolePts.m_stPts.emplace_back(PointXYZ{pt.x, pt.y, pt.z});
    }

    if (m_stMapPolePts.m_stPts.empty())
    {
        m_bUsePolePts = false;
    }
    else
    {
        m_bUsePolePts = true;
        m_stMapPolePts.BuildFLANNIndex();
    }
    m_mtxMapFeatures.unlock();
}

/**@brief publish detected feature points;
 *
 * publish detected feature points;
 * @param timestamp [IN]: the timestamp;
 * @param marker_pts [IN]: the marker points;
 * @param edge_pts [IN]: the edge points;
 * @param pole_pts [IN]: the pole points;
 * @note the feature points are published in vehicle frame;
 */
void Localizer::PublishDetectedFeaturePts(const ros::Time &timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts)
{
    static auto pub_poles = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("detected_pole_pts", 10);
    static auto pub_edges = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("detected_edge_pts", 10);
    static auto pub_markers = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("detected_marker_pts", 10);

    sensor_msgs::PointCloud msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = "/rslidar";
    geometry_msgs::Point32 point;

    msg.points.clear();
    msg.points.reserve(edge_pts.size());
    for (auto &pt:edge_pts)
    {
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        msg.points.emplace_back(point);
    }
    pub_edges.publish(msg);


    msg.points.clear();
    msg.points.reserve(pole_pts.size());
    for (const auto &pt:pole_pts)
    {
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        msg.points.emplace_back(point);
    }
    pub_poles.publish(msg);

    msg.points.clear();
    msg.points.reserve(marker_pts.size());
    for (const auto &pt:marker_pts)
    {
        point.x = pt.x();
        point.y = pt.y();
        point.z = pt.z();
        msg.points.emplace_back(point);
    }
    pub_markers.publish(msg);
}

/**@brief publish localized feature points;
 *
 * publish localized feature points;
 * @param timestamp [IN]: the timestamp;
 * @param marker_pts [IN]: the marker points in vehicle frame;
 * @param edge_pts [IN]: the edge points in vehicle frame;
 * @param pole_pts [IN]: the pole points in vehicle frame;
 * @param pose [IN]: the localized vehicle pose;
 * @note the localized feature points are published in engineer frame;
 */
void Localizer::PublishLocalizedFeaturePts(const ros::Time &timestamp, const PointsVector<PointXYZI> &marker_pts, const PointsVector<PointXYZ> &edge_pts, const PointsVector<PointXYZ> &pole_pts, const Vector3d &pose)
{
    static auto pub_poles = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("localized_pole_pts", 10);

    static auto pub_edges = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("localized_edge_pts", 10);

    static auto pub_markers = m_pNodeHandle->advertise<sensor_msgs::PointCloud>("localized_marker_pts", 10);

    geometry_msgs::Point32 pt;
    sensor_msgs::PointCloud msg;
    double cos_yaw = cos(pose(2));
    double sin_yaw = sin(pose(2));

    msg.header.stamp = timestamp;
    msg.header.frame_id = "/engineer";

    msg.points.clear();
    msg.points.reserve(pole_pts.size());
    for (int p = 0; p < pole_pts.size(); ++p)
    {
        pt.x = cos_yaw * (pole_pts[p].x() + 1.3) - sin_yaw * pole_pts[p].y() + pose(0);
        pt.y = sin_yaw * (pole_pts[p].x() + 1.3) + cos_yaw * pole_pts[p].y() + pose(1);
        pt.z = 0.0;
        msg.points.emplace_back(pt);
    }
    pub_poles.publish(msg);


    msg.points.clear();
    msg.points.reserve(edge_pts.size());
    for (int p = 0; p < edge_pts.size(); ++p)
    {
        pt.x = cos_yaw * (edge_pts[p].x() + 1.3) - sin_yaw * edge_pts[p].y() + pose(0);
        pt.y = sin_yaw * (edge_pts[p].x() + 1.3) + cos_yaw * edge_pts[p].y() + pose(1);
        pt.z = 0.0;
        msg.points.emplace_back(pt);
    }
    pub_edges.publish(msg);

    msg.points.clear();
    msg.points.reserve(marker_pts.size());
    for (int p = 0; p < marker_pts.size(); ++p)
    {
        pt.x = cos_yaw * (marker_pts[p].x() + 1.3) - sin_yaw * marker_pts[p].y() + pose(0);
        pt.y = sin_yaw * (marker_pts[p].x() + 1.3) + cos_yaw * marker_pts[p].y() + pose(1);
        pt.z = 0.0;
        msg.points.emplace_back(pt);
    }
    pub_markers.publish(msg);
}

/**@brief publish particles for visualization;
 *
 * @param timestamp [IN]: the timestamp;
 */
void Localizer::PublishParticles(const ros::Time &timestamp)
{
    static auto pub_particles = m_pNodeHandle->advertise<visualization_msgs::MarkerArray>("particles", 10);
    visualization_msgs::MarkerArray particle_array;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/engineer";
    marker.header.stamp = timestamp;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::ARROW;

    particle_array.markers.reserve(FLAGS_num_imu + FLAGS_num_gps);

    marker.lifetime = ros::Duration();
    marker.ns = "particles";
    m_mtxParticles.lock_shared();
    int id = 0;
    for (const auto &particle:m_vecParticles)
    {
        marker.id = id++;
        marker.pose.position.x = particle.m_evPose(0);
        marker.pose.position.y = particle.m_evPose(1);
        marker.pose.position.z = 0.0;

        tf::Quaternion quaternion;
        quaternion.setRPY(0.0, 0.0, particle.m_evPose(2));

        marker.pose.orientation.x = quaternion.x();
        marker.pose.orientation.y = quaternion.y();
        marker.pose.orientation.z = quaternion.z();
        marker.pose.orientation.w = quaternion.w();

        marker.scale.x = 0.1;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;

        marker.color.b = particle.m_fWeight;
        marker.color.a = particle.m_fWeight;

        particle_array.markers.emplace_back(marker);
    }
    m_mtxParticles.unlock_shared();

    pub_particles.publish(particle_array);
}

/**@brief publish particle trajectory for debug;
 *
 * publish vehicle trajectory of best particle in last 20 seconds;
 * @param pose [IN]: current vehicle pose;
 * @param header [IN]: message header;
 * @note the path are published in engineer frame;
 */
void Localizer::PublishParticleTrajectory(const ros::Time &timestamp, const Vector3d &pose)
{
    static auto pub_traj = m_pNodeHandle->advertise<nav_msgs::Path>("particle_traj", 10);
    static double last_timestamp = timestamp.toSec();
    static deque<Vector3d> poses;
    if (timestamp.toSec() < last_timestamp)
    {
        poses.clear();
    }

    poses.push_back(pose);
    if (poses.size() > 200)
    {
        poses.pop_front();
    }

    nav_msgs::Path path;
    path.header.stamp = timestamp;
    path.header.frame_id = "/engineer";
    path.poses.reserve(poses.size());
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation.w = 1.0;

    for (const auto &pose:poses)
    {
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        path.poses.emplace_back(pose_msg);
    }
    pub_traj.publish(path);

    last_timestamp = timestamp.toSec();
}

/**@brief publish gps trajectory;
 *
 * publish gps trajectory reported by gnss;
 * @param timestamp [IN]: the timestamp;
 * @param gps_x [IN]: the x coordinate in engineer frame;
 * @param gps_y [IN]: the y coordinate in engineer frame;
 */
void Localizer::PublishGpsTrajectory(const ros::Time &timestamp, double gps_x, double gps_y)
{
    static auto pub_traj = m_pNodeHandle->advertise<nav_msgs::Path>("gps_traj", 10);
    if (pub_traj.getNumSubscribers() == 0)
    {
        return;
    }

    static double last_timestamp = timestamp.toSec();
    double current_timestamp = timestamp.toSec();
    static deque<Vector2d> poses;
    if (current_timestamp < last_timestamp)
    {
        poses.clear();
    }

    Vector2d p;
    p << gps_x, gps_y;
    poses.push_back(p);
    if (poses.size() > 100)
    {
        poses.pop_front();
    }

    nav_msgs::Path path;
    path.header.stamp = timestamp;
    path.header.frame_id = "/engineer";
    path.poses.reserve(poses.size());
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation.w = 1.0;

    for (const auto &pose:poses)
    {
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        path.poses.emplace_back(pose_msg);
    }
    pub_traj.publish(path);

    last_timestamp = current_timestamp;
}

/**@brief publish prior trajectory of ekf filter;
 *
 * publish ekf prior pose trajectory;
 * @param timestamp [IN]: the timestamp;
 * @param pose [IN]: the pose;
 */
void Localizer::PublishPriorTrajectory(const ros::Time &timestamp, const Eigen::Matrix<double, 5, 1> &pose)
{
    static auto pub_traj = m_pNodeHandle->advertise<nav_msgs::Path>("prior_traj", 10);
    if (pub_traj.getNumSubscribers() == 0)
    {
        return;
    }

    static double last_timestamp = timestamp.toSec();
    double cur_timestamp = timestamp.toSec();
    static deque<Vector3d> poses;
    if (cur_timestamp < last_timestamp)
    {
        poses.clear();
    }

    Vector3d p;
    p << pose(0), pose(1), pose(3);
    poses.push_back(p);
    if (poses.size() > 1000)
    {
        poses.pop_front();
    }

    nav_msgs::Path path;

    path.header.stamp = timestamp;
    path.header.frame_id = "/engineer";
    path.poses.reserve(poses.size());
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation.w = 1.0;

    for (const auto &pose:poses)
    {
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        path.poses.emplace_back(pose_msg);
    }
    pub_traj.publish(path);

    last_timestamp = cur_timestamp;
}

/**@brief publish posterior trajectory of ekf filter;
 *
 * publish posterior pose of ekf filter;
 * @param timestamp [IN]: the timestamp;
 * @param pose [IN]: the pose to be published;
 */
void Localizer::PublishPosteriorTrajectory(const ros::Time &timestamp, const Eigen::Matrix<double, 5, 1> &pose)
{
    static auto pub_traj = m_pNodeHandle->advertise<nav_msgs::Path>("posterior_traj", 10);
    if (pub_traj.getNumSubscribers() == 0)
    {
        return;
    }

    static double last_timestamp = timestamp.toSec();
    static deque<Vector3d> poses;
    if (timestamp.toSec() < last_timestamp)
    {
        poses.clear();
    }

    Vector3d p;
    p << pose(0), pose(1), pose(3);
    poses.push_back(p);
    if (poses.size() > 500)
    {
        poses.pop_front();
    }

    nav_msgs::Path path;
    path.header.stamp = timestamp;
    path.header.frame_id = "/engineer";
    path.poses.reserve(poses.size());
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.orientation.w = 1.0;

    for (const auto &pose:poses)
    {
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        path.poses.emplace_back(pose_msg);
    }
    pub_traj.publish(path);

    last_timestamp = timestamp.toSec();
}

/**@brief publish localized vehicle pose;
 *
 * publish localized vehicle pose by publishing transform between engineer and localized_vehicle;
 * @param pose [IN]: the pose to be published;
 * @param stamp [IN]: the timestamp of the pose;
 */
void Localizer::PublishLocalizedPose(const ros::Time &stamp, const Vector3d &pose)
{
    static tf::TransformBroadcaster tf_broadcaster;

    tf::Transform transform;
    transform.setRotation(tf::Quaternion(0, 0, 0, 1.0));
    transform.setOrigin(tf::Vector3(pose(0), pose(1), 0.0));
    tf_broadcaster.sendTransform(tf::StampedTransform{transform, stamp, "/engineer", "/localized_vehicle"});
}

/**@brief publish prior pose to CAINIAO;
 *
 * publish ekf prior pose to CAINIAO;
 * @param stamp [IN]: the time stamp;
 * @param pose [IN]: the pose, in order utm_x, utm_y, utm_z, heading;
 */
void Localizer::PublishPriorPose(const ros::Time &stamp, const Vector4d &pose)
{
    static auto pub_odom = m_pNodeHandle->advertise<nav_msgs::Odometry>("/autogo/localization/odometry/filtered_odom", 10);

    double utm_x = pose(0) - FLAGS_utm_x;
    double utm_y = pose(1) - FLAGS_utm_y;

    nav_msgs::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "/map";
    msg.pose.pose.position.x = utm_x;
    msg.pose.pose.position.y = utm_y;
    msg.pose.pose.position.z = pose(2);
    auto quat = tf::createQuaternionFromYaw(pose(3));
    msg.pose.pose.orientation.x = quat.x();
    msg.pose.pose.orientation.y = quat.y();
    msg.pose.pose.orientation.z = quat.z();
    msg.pose.pose.orientation.w = quat.w();

    /**@note simply copy the wheel speed*/
    m_mtxWheelSpeed.lock_shared();
    msg.twist.twist.linear.x = m_fWheelSpeed;
    m_mtxWheelSpeed.unlock_shared();

    /**@note simply copy the imu yaw velocity*/
    m_mtxYawVelocity.lock();
    msg.twist.twist.angular.z = m_fYawVelocity;
    m_mtxYawVelocity.unlock();
    pub_odom.publish(msg);

    static tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;
    transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    transform.setOrigin(tf::Vector3(utm_x, utm_y, pose(2)));
    tf_broadcaster.sendTransform(tf::StampedTransform{transform, stamp, "/rs_odom", "/base_link"});

    quat = tf::createQuaternionFromYaw(FLAGS_shift_yaw);
    transform.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
    transform.setOrigin(tf::Vector3(FLAGS_shift_x, FLAGS_shift_y, 0.0));
    tf_broadcaster.sendTransform(tf::StampedTransform{transform, stamp, "/map", "/rs_odom"});
}
