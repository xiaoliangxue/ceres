#include "global.hpp"
#include "Localizer.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

struct LIDAR_POINT
{
    float fX;
    float fY;
    float fZ;
    float fI;
};

void CallbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pts)
{
    sensor_msgs::PointCloud scan;
    sensor_msgs::convertPointCloud2ToPointCloud(*pts, scan);

    FILE* fp = fopen((GetCurrentDateTimeString()+string(".asc")).data(), "w");
    cerr<<pts->header.stamp.toSec()<<"\t"<<pts->header.seq<<endl;

//    shared_ptr<LIDAR_POINT> scan{new LIDAR_POINT[pts->width], [](LIDAR_POINT* ptr){delete[](ptr);}};

//    LIDAR_POINT* scan = (LIDAR_POINT*)(pts->data.data());

    auto num_of_pts = scan.points.size();

    for (int i = 0; i < num_of_pts; ++i)
    {
        fprintf(fp, "%.3f\t%.3f\t%.3f\t%.3f\t%d\n", scan.points[i].x, scan.points[i].y, scan.points[i].z, scan.channels[0].values[i], i%pts->width);
//        fprintf(fp, "%.3f\t%.3f\t%.3f\t%.3f\n", scan[i].fX, scan[i].fY, scan[i].fZ, scan[i].fI);
    }

    fprintf(fp, "\n");
    fflush(fp);
    fclose(fp);
}

int main(int argc, char** argv)
{
    google::SetVersionString("0.8.0");
    google::SetUsageMessage(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = std::string("./");

    ros::init(argc, argv, "localizer");

//    ros::NodeHandle node;
//
//    ros::Subscriber subscriber = node.subscribe("/middle/rslidar_points", 10, CallbackPointCloud);

    Localizer localizer;

    ros::spin();
    return (EXIT_SUCCESS);

}

