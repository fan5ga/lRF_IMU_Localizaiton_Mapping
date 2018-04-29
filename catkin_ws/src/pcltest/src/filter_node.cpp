#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/model_outlier_removal.h>


int main(int argc, char** argv)
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in1 (new pcl::PointCloud<pcl::PointXYZ>);// (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZ>);// (new pcl::PointCloud<pcl::PointXYZ>);
    std::stringstream ss0;
    ss0 << "/home/wangzt/paper/11.pcd";//<< argv[1] << ".pcd";
//    std::cout<<argv[1]<<argv[2]<<argv[3]<<argv[4]<<argv[5]<<std::endl;
    if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss0.str (), *cloud_in)==-1)
    {
        return 5;
    }
//    std::stringstream ss01;
//    ss01 << "/home/wangzt/hallway-slow-ground/zuizhong/left-final2.pcd";//<< argv[2] << ".pcd";

//    if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss01.str (), *cloud_in2)==-1)
//    {
//        return 5;
//    }


    //////////////////////////////transformation/////////////////////////////////////////
//    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

////    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
//    double theta = M_PI* 0/180;// M_PI* atof(argv[1])/180;  // The angle of rotation in radians
//    transformation_matrix (0, 0) = cos (theta);
//    transformation_matrix (0, 1) = -sin (theta);
//    transformation_matrix (1, 0) = sin (theta);
//    transformation_matrix (1, 1) = cos (theta);

//    // A translation on Z axis (0.4 meters)
////    transformation_matrix (0, 3) = 0.2;//-2.4;//atof(argv[2]);//-0.8;//0.4;
////    transformation_matrix (1, 3) = -0.1;//15.5;//atof(argv[3]);//-0.8;//0.4;
//        transformation_matrix (2, 3) = 0.08;//15.5;//atof(argv[3]);//-0.8;//0.4;

//    // Executing the transformation
//    pcl::transformPointCloud (*cloud_in, *cloud_out, transformation_matrix);

//    *cloud_out = *cloud_in1 + *cloud_f;
//    *cloud_in = *cloud_in1 + *cloud_in2;

    ////////////////////////////////box filter/////////////////////////////////////////
//    pcl::PCLPointCloud2::Ptr cloud2_in (new pcl::PCLPointCloud2 ());
//    pcl::PCLPointCloud2::Ptr cloud2_out (new pcl::PCLPointCloud2 ());
//    pcl::toPCLPointCloud2(*cloud_in, *cloud2_in);
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor0;
//    sor0.setInputCloud (cloud2_in);
//    sor0.setLeafSize (0.02f, 0.02f, 0.02f);
//    sor0.filter (*cloud2_out);
//    pcl::fromPCLPointCloud2(*cloud2_out, *cloud_out);


/////////////////////////////////////////pass through////////////////////////////////
//    pcl::PassThrough<pcl::PointXYZ> pass;
//    pass.setInputCloud (cloud_in);
//    pass.setFilterFieldName ("z");
//    pass.setFilterLimits (-2,2);
//    pass.setFilterLimitsNegative (true);
//    pass.filter (*cloud_out);
//    pass.setInputCloud (cloud_out);
//    pass.setFilterFieldName ("y");
//    pass.setFilterLimits (-16.6,-2.8);
//    pass.filter (*cloud_out);
//    pass.setFilterLimitsNegative (false);
//    pass.setInputCloud (cloud_out);
//    pass.setFilterFieldName ("x");
//    pass.setFilterLimits (-28.6,-16.2);
//    pass.filter (*cloud_out);
//    std::stringstream sts;
//    sts << "/home/wangzt/2d-hallway/" << argv[1] << "-.pcd";
//    pcl::io::savePCDFile(sts.str (), *cloud_out);
//    pcl::io::savePCDFile("/home/wangzt/2d-hallway/up5.pcd", *cloud_out);

////////////////////// outliers//////////////////////////////////
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//    sor.setInputCloud (cloud_in);
//    sor.setMeanK (9);
//    sor.setStddevMulThresh (0.8);
////    sor.setNegative(true);
//    sor.filter (*cloud_out);



//    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
//    // build the filter
//    outrem.setInputCloud(cloud_in);
//    outrem.setRadiusSearch(0.05);
//    outrem.setMinNeighborsInRadius (10);
//    // apply filter
//    outrem.filter (*cloud_out);

//    pcl::io::savePCDFile("/home/wangzt/paper/0_1_outlier.pcd", *cloud_out);
///////////////////////////////////////////////////////////////////////////////////////////////
//    pcl::ModelCoefficients sphere_coeff;
//    sphere_coeff.values.resize (4);
//    sphere_coeff.values[0] = 0;
//    sphere_coeff.values[1] = 0;
//    sphere_coeff.values[2] = 0;
//    sphere_coeff.values[3] = 1;

//    pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
//    sphere_filter.setModelCoefficients (sphere_coeff);
//    sphere_filter.setThreshold (0.05);
//    sphere_filter.setModelType (pcl::SACMODEL_SPHERE);
//    sphere_filter.setInputCloud (cloud);
//    sphere_filter.filter (*cloud_sphere_filtered);


////////////////////////////plane///////////////////////////////////////////////////////////
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);  //pcl::SACMODEL_PLANE
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setDistanceThreshold (0.1);

//    seg.setInputCloud (cloud_in);
//    seg.segment (*inliers, *coefficients);


//    if (inliers->indices.size () == 0)
//    {
//      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
//      return (2);
//    }

//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//                                        << coefficients->values[1] << " "
//                                        << coefficients->values[2] << " "
//                                        << coefficients->values[3] << std::endl;

//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;


//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud (cloud_in);
//    extract.setIndices (inliers);
//    extract.setNegative (0);
//    extract.filter (*cloud_out);


//    pcl::io::savePCDFile("/home/wangzt/wode1_plane.pcd", *cloud_f);

/////////////////////////////////////////////EuclideanClusterExtraction/////////////////////////////////
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (3000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_in);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_out->points.push_back (cloud_in->points[*pit]); //*
      cloud_out->width = cloud_out->points.size ();
      cloud_out->height = 1;
      cloud_out->is_dense = true;

      std::cout << "PointCloud representing the Cluster: " << cloud_out->points.size () << " data points_"<< j << std::endl;
      std::stringstream ss;
      ss << "/home/wangzt/paper/11" <<'-'<<j << ".pcd";
      pcl::io::savePCDFile(ss.str (), *cloud_out);
      cloud_out->clear();
      j++;

    }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    int v1 (0);
//    int v2 (1);
//    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
//    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

//    viewer.setBackgroundColor (0.0, 0.0, 0.0, v1);
//    viewer.setBackgroundColor (0.0, 0.0, 0.0, v2);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_fil_color_h (cloud_out, 255.0, 255.0, 255.0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_row_color_h (cloud_in, 255.0, 255.0, 255.0);
//    viewer.addPointCloud (cloud_in, cloud_fil_color_h, "filtered", v2);
//    viewer.addPointCloud (cloud_out, cloud_row_color_h, "1n", v1);
//    std::strstream ss;
//    ss<<cloud_out->size();
//    std::string scloud1;// = cloud2->size();
//    ss>>scloud1;
//    viewer.addText (scloud1, 10, 15, 16, 100, 100, 100, "icp_info_1", v1);
//    std::strstream ss1;
//    ss1<<cloud_in->size();
//    std::string scloud2;// = cloud2->size();
//    ss1>>scloud2;
//    viewer.addText (scloud2, 10, 15, 16, 100, 100, 100, "icp_info_2", v2);

//    while (!viewer.wasStopped ())
//    {
//      viewer.spinOnce ();
//    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> aa;
//    aa.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));

//    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/paper/10-0.pcd", *basic_cloud_ptr)==-1)
//    {
//        return 5;
//    }

//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/paper/10-1.pcd", *temp_cloud)==-1)
//    {
//        return 5;
//    }
//    aa[0] = basic_cloud_ptr;
//    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
//    viewer.setBackgroundColor (0.0, 0.0, 0.0);

//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(basic_cloud_ptr, 255, 0, 0);
//    viewer.addPointCloud<pcl::PointXYZ> (aa[0], single_color, "sample cloud");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(temp_cloud, 0, 255, 0);
//    viewer.addPointCloud<pcl::PointXYZ> (temp_cloud, single_color2, "sample cloud1");

//    //viewer->addCoordinateSystem (1.0);
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
//    viewer.initCameraParameters ();


//    while (!viewer.wasStopped ())
//    {
//      viewer.spinOnce (100);
//      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//    }
    return (0);



}

//int
//main ()
//{
//    pcl::PCLPointCloud2::Ptr cloud1 (new pcl::PCLPointCloud2 ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered1 (new pcl::PointCloud<pcl::PointXYZ>);

//    // Fill in the cloud data
//    pcl::PCDReader reader;
//    // Replace the path below with the path where you saved your file
////    reader.read<pcl::PointXYZ> ("/home/catkin_ws/reference_downsampled.pcd", *cloud);
//    reader.read ("/home/wangzt/catkin_ws/table_scene_lms400_downsampled.pcd", *cloud1);
//    pcl::fromPCLPointCloud2(*cloud1, *cloud);
//    std::cerr << "Cloud before filtering: " << std::endl;
//    std::cerr << *cloud << std::endl;


////    pcl::PCDWriter writer;
////    pcl::toPCLPointCloud2(*cloud_filtered, *cloud1);
////    writer.write ("/home/wangzt/catkin_ws/table_scene_lms400_inliers1.pcd", *cloud1, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (),false);
//    pcl::io::savePCDFile("/home/wangzt/catkin_ws/test1_done.pcd", *cloud_filtered, false);

//    sor.setNegative (true);
//    sor.filter (*cloud_filtered1);
////    pcl::toPCLPointCloud2(*cloud_filtered, *cloud1);
////    writer.write ("/home/wangzt/catkin_ws/table_scene_lms400_outliers1.pcd", *cloud1, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);


//}

