#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <vector>
#include <ctime>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

  std::ofstream fd("/home/wangzt/paper/0_1.txt");
  std::stringstream ss0;
  ss0 << "/home/wangzt/paper/0_1.pcd";//<< argv[1] << ".pcd";
  if(pcl::io::loadPCDFile<pcl::PointXYZ>(ss0.str (), *cloud)==-1)
  {
      return 5;
  }

  kdtree.setInputCloud (cloud);

  pcl::PointXYZ searchPoint;

  // K nearest neighbor search

  int K = 9;
  double dista=0;

  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  for(size_t p = 0; p < cloud->points.size(); p++)
  {
      searchPoint.x = cloud->points[p].x;
      searchPoint.y = cloud->points[p].y;
      searchPoint.z = cloud->points[p].z;

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
            if(i!=0)
            {
                dista += sqrt(pointNKNSquaredDistance[i]);
//                std::cout<< "***" << pointNKNSquaredDistance[i] <<std::endl;
            }
        }
        dista = dista/(K-1);
        std::cout<< p << ":" << dista <<std::endl;
        fd << dista << std::endl;
      }
      dista = 0;

  }
  fd.close();

  // Neighbors within radius search

//  std::vector<int> pointIdxRadiusSearch;
//  std::vector<float> pointRadiusSquaredDistance;

//  float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

//  std::cout << "Neighbors within radius search at (" << searchPoint.x
//            << " " << searchPoint.y
//            << " " << searchPoint.z
//            << ") with radius=" << radius << std::endl;


//  if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
//  {
//    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//      std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
//                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
//                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
//                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//  }


  return 0;
}
