#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char** argv)
{
    double x_min=-16.2;
    double x_max=13.9;
    double y_min=17.3;
    double y_max=18.7;
//    double z_min=0;
//    double z_max=0;
    pcl::PointCloud<pcl::PointXYZ> cloud; //(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud1;// (new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::io::loadPCD<pcl::PointXYZ>( "/home/wangzt/2d-hallway/inside-final.pcd", cloud);
//   // for(int j=0; j<37;j++)
    {
    std::stringstream ss;
    ss << "/home/wangzt/2d-hallway/new-left.pcd";// <<argv[1]<<".pcd";//inside-final.pcd";// << j << ".pcd";//<< ".pcd";//

    if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss.str(), cloud1)==-1)
    {
        return 5;
    }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/new-right.pcd", *cloud2)==-1)
        {
            return 5;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3 (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/new-down.pcd", *cloud3)==-1)
        {
            return 5;
        }
        pcl::PointCloud<pcl::PointXYZ> cloud4;// (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/new-up.pcd", cloud4)==-1)
        {
            return 5;
        }
//        pcl::PointCloud<pcl::PointXYZ> cloud5;// (new pcl::PointCloud<pcl::PointXYZ>);
//        if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/wangzt/2d-hallway/left.pcd", cloud5)==-1)
//        {
//            return 5;
//        }
        cloud += cloud1;
        cloud += *cloud2;
        cloud += *cloud3;
        cloud += cloud4;
//        cloud += cloud5;
    }
        pcl::io::savePCDFile("/home/wangzt/2d-hallway/new-final.pcd", cloud);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    std::cout << cloud1.size() << std::endl;
//////    std::cout << cloud2.size() << std::endl;
//    x_min = cloud1.points[0].x;
//    x_max = cloud1.points[0].x;
//    y_max=cloud1.points[0].y ;
//    y_min=cloud1.points[0].y ;
//    for(int i; i<cloud1.size(); i++)
//    {
//        if(x_max<cloud1.points[i].x)  x_max=cloud1.points[i].x ;
//        if(x_min>cloud1.points[i].x)  x_min=cloud1.points[i].x ;
//        if(y_max<cloud1.points[i].y)  y_max=cloud1.points[i].y ;
//        if(y_min>cloud1.points[i].y)  y_min=cloud1.points[i].y ;

////        y+=cloud1.points[i].y ;
////        z+=cloud1.points[i].z ;
//    }
//    std::cout << x_min<<',' << x_max << y_min <<','<< y_max <<std::endl;
//    cloud.width = (int)((y_max-y_min)/0.1);
//    cloud.height = (int)((x_max-x_min)/0.1);
//    cloud.resize(cloud.width*cloud.height);
//    std::cout << cloud.width<<',' <<cloud.height <<','<< cloud.size() <<std::endl;
//    int i = 0;
//    int k = 0;
//    while( i<cloud.height)
//    {
//        std::cout<< i<<','<<x_min + (double)((double)i*0.05)<<std::endl;
//        for(k; k<cloud.width; k++)
//        {
//            std::cout<< i<<std::endl;
//            cloud.points[k+i*cloud.width].x = x_min + (double)((double)i*0.1);
//            cloud.points[k+i*cloud.width].y = y_min + (double)((double)k*0.1);
//            cloud.points[k+i*cloud.width].z = 0;
//        }
//        k=0;
//        i++;
//    }
//    std::stringstream ss1;
//    ss1 << "/home/wangzt/hallway-slow-ground/zuizhong/ground_up.pcd";// << argv[1] << ".pcd";

//    pcl::io::savePCDFile(ss1.str(), cloud);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//        x_min = cloud1->points[0].x;
//        x_max = cloud1->points[0].x;
//        y_max=cloud1->points[0].y ;
//        y_min=cloud1->points[0].y ;
//        for(int i; i<cloud1->size(); i++)
//        {
//            if(y_max<cloud1->points[i].y)  y_max=cloud1->points[i].y ;
//            if(y_min>cloud1->points[i].y)  y_min=cloud1->points[i].y ;
//            if(x_max<cloud1->points[i].x)  x_max=cloud1->points[i].x ;
//            if(x_min>cloud1->points[i].x)  x_min=cloud1->points[i].x ;

//        }

//        std::cout << x_min<<',' << x_max <<','<< y_min <<','<< y_max <<cloud1->size()<<std::endl;
//        pcl::PassThrough<pcl::PointXYZ> pass;
//        pass.setInputCloud (cloud2);                                                  //        pass.setFilterLimitsNegative (true);
//        pass.setFilterFieldName ("y");
//        pass.setFilterLimits (y_min-0.1, y_max+0.1);
//        pass.filter (*cloud);
//        pass.setInputCloud (cloud);
//        pass.setFilterFieldName ("x");
//        pass.setFilterLimits (x_min-0.1, x_max+0.1);
//        pass.filter (*cloud2);
//        std::stringstream ss1;
//        ss1 << "/home/wangzt/hallway-slow-ground/" << argv[1] << ".pcd";

//        pcl::io::savePCDFile(ss1.str(), *cloud2);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//        x_min = cloud1->points[0].x;
//        x_max = cloud1->points[0].x;
//        y_max=cloud1->points[0].y ;
//        y_min=cloud1->points[0].y ;
//        z_max=cloud1->points[0].z ;
//        z_min=cloud1->points[0].z ;
//        std::cout << x_min<<',' << x_max <<','<< y_min <<','<< y_max <<','<< z_min <<','<< z_max <<std::endl;
//        for(int i; i<cloud1->size(); i++)
//        {
//     //       std::cout << i <<std::endl;
//            double x,y,z;
//            x = cloud1->points[i].x ;
//            y = cloud1->points[i].y ;
//            z = cloud1->points[i].z ;
//            if(y_max<y)  y_max=y ;
//            if(y_min>y)  y_min=y ;
//            if(x_max<x)  x_max=x ;
//            if(x_min>x)  x_min=x ;
//            if(z_max<z)  z_max=z ;
//            if(z_min>z)  z_min=z ;

//        }

//        std::cout << x_min<<',' << x_max <<','<< y_min <<','<< y_max <<','<< z_min <<','<< z_max <<std::endl;
//        if((x_max-x_min)<(y_max-y_min))
//        {
//            std::cout << 'x' <<std::endl;
//            cloud->width = (int)((y_max-y_min)/0.05);
//            cloud->height = (int)((z_max-z_min)/0.05);
//            cloud->resize(cloud->width*cloud->height);
//   //         std::cout << cloud.width<<',' <<cloud.height <<','<< cloud.size() <<std::endl;
//            int i = 0;
//            int k = 0;
//            while( i<cloud->height)
//            {
//    //            std::cout<< i<<','<<x_min + (double)((double)i*0.05)<<std::endl;
//                for(k; k<cloud->width; k++)
//                {
//    //                std::cout<< i<<std::endl;
//                    cloud->points[k+i*cloud->width].z = z_min + (double)((double)i*0.05);
//                    cloud->points[k+i*cloud->width].y = y_min + (double)((double)k*0.05);
//                    cloud->points[k+i*cloud->width].x = (x_max+x_min)*0.5;
//                }
//                k=0;
//                i++;
//            }

//        }
//        else
//        {
//            std::cout << 'y' <<std::endl;
//            cloud->width = (int)((z_max-z_min)/0.05);
//            cloud->height = (int)((x_max-x_min)/0.05);
//            cloud->resize(cloud->width*cloud->height);
//       //     std::cout << cloud.width<<',' <<cloud.height <<','<< cloud.size() <<std::endl;
//            int i = 0;
//            int k = 0;
//            while( i<cloud->height)
//            {
//                //            std::cout<< i<<','<<x_min + (double)((double)i*0.05)<<std::endl;
//                for(k; k<cloud->width; k++)
//                {
//                    //                std::cout<< i<<std::endl;
//                    cloud->points[k+i*cloud->width].x = x_min + (double)((double)i*0.05);
//                    cloud->points[k+i*cloud->width].z = z_min + (double)((double)k*0.05);
//                    cloud->points[k+i*cloud->width].y = (y_max+y_min)*0.5;
//                }
//                k=0;
//                i++;
//            }
//        }
//        std::stringstream ss1;
//        ss1 << "/home/wangzt/hallway-slow-ground/qiang/fen/final/" << argv[1] << ".pcd";

//        pcl::io::savePCDFile(ss1.str(), *cloud);

    return 1;


}



