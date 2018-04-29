#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

#define bufLen 1000

bool next_iteration = false;
static char buffer[bufLen];


void
print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    printf ("r = < %6.3f >\n\n", (180/3.1415926)*atan2(matrix (1, 0),matrix (0, 0)));
}

void
keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing)
{
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

int
main (int argc,   char* argv[])
{
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/raw.pcd", *cloud_icp)==-1)
    {
        return 5;
    }
    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/final.pcd", *cloud_in)==-1)
    {
        return 5;
    }

//    cloud_in->width = 9;
//    cloud_in->height = 1;
//    cloud_in->resize(cloud_in->width*cloud_in->height);
//    cloud_tr->width = 9;
//    cloud_tr->height = 1;
//    cloud_tr->resize(cloud_tr->width*cloud_tr->height);
//    cloud_icp->width = 9;
//    cloud_icp->height = 1;
//    cloud_icp->resize(cloud_icp->width*cloud_icp->height);

///////////////////////////////////////////////////////////////////////////////////////////////////
//    ifstream file("/home/wangzt/catkin_ws/out1.txt",ios::in);
//    if(!file)return 0;

//    const char *delima = "(, )";

//    double point1[100][3];
//    double point2[100][3];
//    int num1 = 0;

//    file.getline(buffer,bufLen,'\n');
//    while(!file.eof())
//    {
//        char *p;
//        p = strtok(buffer, delima);
//        double temp;
//        int num2 = 0;

//        while(p)
//        {
//            temp = atof(p);
//            if(num2<3) point1[num1][num2] = temp;
//            else point2[num1][num2-3] = temp;
//            p=strtok(NULL,delima);
//            num2++;
//        }
//        file.getline(buffer,bufLen,'\n');
//        num1++;
//    }
//    for(size_t i = 0; i < cloud_in->size();++i)
//    {
//        cloud_in->points[i].x = point1[i][0];
//    }
//    for(size_t i = 0; i < cloud_in->size();++i)
//    {
//        cloud_in->points[i].y = point1[i][1];
//    }
//    for(size_t i = 0; i < cloud_in->size();++i)
//    {
//        cloud_in->points[i].z = point1[i][2];
//    }
//    for(size_t i = 0; i < cloud_icp->size();++i)
//    {
//        cloud_icp->points[i].x = point2[i][0];
//    }
//    for(size_t i = 0; i < cloud_icp->size();++i)
//    {
//        cloud_icp->points[i].y = point2[i][1];
//    }
//    for(size_t i = 0; i < cloud_icp->size();++i)
//    {
//        cloud_icp->points[i].z = point2[i][2];
//    }

//////////////////////////////////////////////////////////////////////////////////////////////
    // Checking program arguments
    //  if (argc < 2)
    //  {
    //    printf ("Usage :\n");
    //    printf ("\t\t%s file.ply number_of_ICP_iterations\n", argv[0]);
    //    PCL_ERROR ("Provide one ply file.\n");
    //    return (-1);
    //  }

    int iterations = 1;  // Default number of ICP iterations
    //  if (argc > 2)
    //  {
    //    // If the user passed the number of iteration as an argument
    //    iterations = atoi (argv[2]);
    //    if (iterations < 1)
    //    {
    //      PCL_ERROR ("Number of initial iterations must be >= 1\n");
    //      return (-1);
    //    }
    //  }

    pcl::console::TicToc time;
    time.tic ();

//    pcl::PCDReader reader;

//    reader.read ("/home/wangzt/catkin_ws/reference.pcd", *cloud_in);
//    reader.read ("/home/wangzt/catkin_ws/1-e.pcd", *cloud_icp);

    //  if (pcl::io::loadPLYFile (argv[1], *cloud_in) < 0)
    //  {
    //    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    //    return (-1);
    //  }
    //std::cout << "\nLoaded file " << argv[1] << " (" << cloud_in->size () << " points) in " << time.toc () << " ms\n" << std::endl;

    // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

    // A rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    double theta = 0;//-M_PI*1 / 8.5;  // The angle of rotation in radians
    transformation_matrix (0, 0) = cos (theta);
    transformation_matrix (0, 2) = sin (theta);
    transformation_matrix (2, 0) = -sin (theta);
    transformation_matrix (2, 2) = cos (theta);

    // A translation on Z axis (0.4 meters)
    transformation_matrix (0, 3) = 0;//0.4;

    // Display in terminal the transformation matrix
    std::cout << "Applying this rigid transformation to: cloud_in -> cloud_icp" << std::endl;
    print4x4Matrix (transformation_matrix);

    // Executing the transformation
    pcl::transformPointCloud (*cloud_in, *cloud_in, transformation_matrix);
    pcl::io::savePCDFile("/home/wangzt/xuanzuan.pcd", *cloud_in);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use

    // The Iterative Closest Point algorithm
    time.tic ();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations (15);//(iterations);
    icp.setInputSource (cloud_icp);
    icp.setInputTarget (cloud_in);
    time.tic ();
    icp.align (*cloud_icp);
   //icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
    std::cout << "Applied1111 " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    // Visualization
    pcl::visualization::PCLVisualizer viewer ("Transformation matix");
    // Create two verticaly separated viewports
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);

    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // Original point cloud is white
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                               (int) 255 * txt_gray_lvl);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // Transformed point cloud is green
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // Adding text descriptions in each viewport
    viewer.addText ("White: Previous Frame\nGreen: Current Frame", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Previous Frame\nRed: After Transformation", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // Set background color
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // Set camera position and orientation
    viewer.setCameraPosition (-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize (1280, 1024);  // Visualiser window size

    // Register keyboard callback :
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // Display the visualiser
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        // The user pressed "space" :
        if (next_iteration)
        {
            // The Iterative Closest Point algorithm
            time.tic ();
            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
                printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose

                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}
