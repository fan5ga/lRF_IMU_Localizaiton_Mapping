#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/io/pcd_io.h>

ros::Publisher pub;
char *file = NULL;

void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//    pcl::PCLPointCloud2 cloud_filtered;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ> ());
    // Convert to PCL data type
    pcl_conversions::toPCL(*input, *cloud);

    // Perform the actual filtering
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud (cloudPtr);
//    sor.setLeafSize (0.1, 0.1, 0.1);
//    sor.filter (cloud_filtered);

    // Convert to ROS data type
//    sensor_msgs::PointCloud2 output;
//    pcl_conversions::fromPCL(cloud_filtered, output);

    // Publish the data
//    pub.publish (output);

  // Create a container for the data.
//  sensor_msgs::PointCloud2 output;

  // Do data processing here...
//  output = *input;

  // Publish the data.
//  pub.publish (output);
//    pcl::fromPCLPointCloud2(*cloud, *cloud1);
//    pcl::io::savePCDFile("/home/wangzt/catkin_ws/new_test/conf5.pcd", *cloud);
    pcl::io::savePCDFile(file, *cloud);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  //  file = argv[1];
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Rate a = 10;

  sensor_msgs::PointCloud2 cloud;
  pcl::PointCloud<pcl::PointXYZ> tem_cloud;
  pcl::PointCloud<pcl::PointXYZ> tem_cloud1;
  pcl::PCLPointCloud2 tem_cloud2;

  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_in", 200);

  std::stringstream ss0;
  ss0 << "/home/wangzt/2dsb/raw.pcd";//<< argv[1] << ".pcd";
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss0.str (), tem_cloud)==-1)
  {
      return 5;
  }
  std::stringstream ss1;
  ss1 << "/home/wangzt/2dsb/x1.pcd";//<< argv[1] << ".pcd";
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss1.str (), tem_cloud1)==-1)
  {
      return 5;
  }
  //tem_cloud+=tem_cloud1;
  pcl::toPCLPointCloud2(tem_cloud, tem_cloud2);
  pcl_conversions::fromPCL(tem_cloud2, cloud);

  cloud.header.frame_id = "map";


  while(1)
  {
      a.sleep();
      pub.publish(cloud);
  }

  // Create a ROS subscriber for the input point cloud
 // ros::Subscriber sub = nh.subscribe ("/octomap_point_cloud_centers", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
//  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
 // ros::spin ();
}
