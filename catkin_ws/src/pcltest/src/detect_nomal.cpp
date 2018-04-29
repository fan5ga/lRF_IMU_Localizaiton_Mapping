#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>


int main()
{

    char trans_str[50];
    sprintf(trans_str,"/home/wangzt/hallway-slow-ground/nomal_ground/%d.txt",0);
    std::ofstream fd(trans_str);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream ss0;
    ss0 << "/home/wangzt/hallway-slow-ground/nomal_ground/hallway-slow-ground.pcd";//<< argv[1] << ".pcd";
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss0.str (), *cloud)==-1)
    {
        return 5;
    }
    std::cout<<cloud->size()<<std::endl;

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setKSearch(8);

    // Compute the features
    ne.compute (*cloud_normals);
    std::cout<<cloud_normals->size()<<std::endl;

     // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // cloud_normals->points.size () should have the same size as the input cloud->points.size ()*
    pcl::io::savePCDFile("/home/wangzt/hallway-slow-ground/nomal_ground/nomaltest.pcd", *cloud_with_normals);
    for(int i=0; i<cloud_with_normals->size(); i++)
    {
        fd<<cloud_with_normals->at(i).curvature<<","<<cloud_with_normals->at(i).normal_x<<","<<cloud_with_normals->at(i).normal_y<<","<<cloud_with_normals->at(i).normal_z<<std::endl;
    }
    fd.close();

 }
