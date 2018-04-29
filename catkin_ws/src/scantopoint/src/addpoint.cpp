#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> /* File control definitions */
#include <errno.h>
#include<sensor_msgs/LaserScan.h>
#include<tf/transform_broadcaster.h>
#include <termios.h>
#include <laser_geometry/laser_geometry.h>

#include<pcl/common/transforms.h>
#include<pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include<pcl/conversions.h>
#include<pcl_conversions/pcl_conversions.h>

laser_geometry::LaserProjection projector;
sensor_msgs::PointCloud2 cloud;
Eigen::Matrix4f cloud_trans = Eigen::Matrix4f::Identity();

pcl::PointCloud<pcl::PointXYZ> cloud_source1;
pcl::PointCloud<pcl::PointXYZ> cloud_source2;
pcl::PCLPointCloud2 tem_cloud;



void scanCallback(ros::Publisher &pub, const sensor_msgs::LaserScan::ConstPtr& scan)
{

    projector.projectLaser(*scan,cloud);
    pcl_conversions::toPCL(cloud, tem_cloud);
    pcl::fromPCLPointCloud2(tem_cloud, cloud_source1);
    cloud_source2 += cloud_source1;
//    pcl::transformPointCloud(cloud_source1, cloud_source2, cloud_trans);
    pcl::toPCLPointCloud2(cloud_source2, tem_cloud);
    pcl_conversions::fromPCL(tem_cloud, cloud);
    pub.publish(cloud);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "addpoint");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_in", 200);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan> ("/scan", 1, boost::bind(&scanCallback, boost::ref(pub), _1));

  cloud_source2.header.frame_id = "/cloud_in";
//  tf::TransformBroadcaster br;
//  tf::Transform transform0;
//  transform0.setOrigin(tf::Vector3(0.095,0,0.48));
//  tf::Quaternion q;
//  cloud_trans(2,3) = 0.56;

while(n.ok())
{

//    q.setRPY(0, -(0.087890625*a-180)/57.3, 0);
//    transform1.setRotation(q1);
//    q1.setRPY(0, 0, 0);
//    transform0.setRotation(q);

//    float th = -(0.087890625*a-180)/57.3;

//    cloud_trans(0,0) = cos(th);
//    cloud_trans(0,2) = sin(th);
//    cloud_trans(2,0) = -sin(th);
//    cloud_trans(2,2) = cos(th);

//  r.sleep();

//  br.sendTransform(tf::StampedTransform(transform0, ros::Time::now(), "/base_footprint", "/cloud"));
  ros::spinOnce();
}
  return(0);
}

