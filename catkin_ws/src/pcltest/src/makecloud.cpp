#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZ> cloud0;// (new pcl::PointCloud<pcl::PointXYZ>);

int
main (int argc, char** argv)
{

    cloud0.width = 3;
    cloud0.height = 3;
    cloud0.resize(9);

    for(int i = 0; i<cloud0.size(); i++)
    {
        cloud0.points[i].x = i;
        cloud0.points[i].y = 0;
        cloud0.points[i].z = 0;

    }

    std::stringstream ss1;
    ss1 << "/home/wangzt/desktop/111.pcd";// << argv[1] << ".pcd";

    pcl::io::savePCDFile(ss1.str(), cloud0);
    return 0;

}
