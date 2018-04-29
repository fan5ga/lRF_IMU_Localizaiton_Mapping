#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>


void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
//    printf ("Rotation matrix :\n");
//    printf ("    | %6.5f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
//    printf ("R = | %6.5f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
//    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
    double r = (180/3.1415926)*atan2(matrix (1, 0),matrix (0, 0));
    printf ("r = < %f >\n\n", r);
}

int main(int argc, char** argv)
{
    int file_cnt = 0;
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> pre_cloud_normals;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pre_cloud;
    std::vector<double> pre_normals_x;
    std::vector<double> pre_normals_y;

    std::stringstream root_str;
    root_str << "/home/wangzt/run/";

    std::ofstream fd2;

    char cmd[50];
    sprintf(cmd, "mkdir /home/wangzt/run/%s", "filted");
    system(cmd);


    while(1)
    {
        std::vector<double> cur_normals_x;
        std::vector<double> cur_normals_y;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cur_cloud;
        std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> cur_cloud_normals;
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

        char trans_str[50];
        sprintf(trans_str,"/home/wangzt/run/txt/%d.txt",file_cnt);
        std::ofstream fd(trans_str);

        pcl::console::TicToc time;


        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filted1 (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filted2 (new pcl::PointCloud<pcl::PointXYZ>);

        std::stringstream raw_data_str;
        raw_data_str<<root_str.str()<<"raw/"<<file_cnt<<".pcd";
        while(pcl::io::loadPCDFile<pcl::PointXYZ>(raw_data_str.str (), *cloud_raw)==-1)
        {
            sleep(1);
        }

        Eigen::Matrix4d transformation_matrix2 = Eigen::Matrix4d::Identity ();

        double theta = -M_PI*1 / 8.5;  // The angle of rotation in radians
        transformation_matrix2 (0, 0) = cos (theta);
        transformation_matrix2 (0, 2) = sin (theta);
        transformation_matrix2 (2, 0) = -sin (theta);
        transformation_matrix2 (2, 2) = cos (theta);

        pcl::transformPointCloud (*cloud_raw, *cloud_raw, transformation_matrix2);


        //filter******************************************************************************************
        time.tic ();
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud (cloud_raw);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.25,2);
  //      pass.setFilterLimitsNegative (true);
        pass.filter (*cloud_filted1);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        outrem.setInputCloud(cloud_filted1);
        outrem.setRadiusSearch(0.05);
        outrem.setMinNeighborsInRadius (30);
        outrem.filter (*cloud_filted2);


        std::cout <<  time.toc () << " ms" << std::endl;

        std::stringstream filted_data_str;
        filted_data_str << root_str.str()<<"filted/"<<file_cnt<<"_filted.pcd";
        pcl::io::savePCDFile(filted_data_str.str(), *cloud_filted2);


        //Euclidean******************************************************************************************

        sprintf(cmd, "mkdir /home/wangzt/run/%d", file_cnt);
        system(cmd);

//        time.tic ();
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
        tree1->setInputCloud (cloud_filted2);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (500);
        ec.setMaxClusterSize (50000);
        ec.setSearchMethod (tree1);
        ec.setInputCloud (cloud_filted2);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ> cloud_temp;
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_temp.points.push_back (cloud_filted2->points[*pit]); //*
            cloud_temp.width = cloud_temp.points.size ();
            cloud_temp.height = 1;
            cloud_temp.is_dense = true;

            std::cout << "PointCloud representing the Cluster: " << cloud_temp.points.size () << " data points_"<< j << std::endl;
            std::stringstream ss;
            ss << root_str.str() << file_cnt <<'/' <<file_cnt <<'-'<<j << ".pcd";
            cur_cloud.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>));
            *cur_cloud.at(cur_cloud.size()-1)=cloud_temp;
            pcl::io::savePCDFile(ss.str (), *cur_cloud.at(cur_cloud.size()-1));

            j++;

        }

//        std::cout <<  time.toc () << " ms" << std::endl;
//        //normal***************************************************************************************************************
////        sprintf(cmd, "mkdir -p /home/wangzt/run/normals/%d/", file_cnt);
////        system(cmd);


//        for(int cloud_i=0; cloud_i<cur_cloud.size(); cloud_i++)
//        {
//            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//            ne.setInputCloud (cur_cloud.at(cloud_i));

//            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ> ());
//            ne.setSearchMethod (tree2);

//            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

//            if(cur_cloud.at(cloud_i)->size()<8000)
//                ne.setKSearch(cur_cloud.at(cloud_i)->size());
//            else
//                ne.setKSearch(5000);
//            //ne.setRadiusSearch(0.3);
//            ne.compute (*cloud_normals);
////            std::cout<<cloud_normals->size()<<std::endl;

//            double avr_normals[2]={0};
//            int k=0;
//            for(int i=0; i<cloud_normals->size(); i+=50)
//            {
//                avr_normals[0]+=cloud_normals->at(i).normal_x;
//                avr_normals[1]+=cloud_normals->at(i).normal_y;
//                k++;
//            }
//            avr_normals[0]/=k;
//            avr_normals[1]/=k;
//            cur_normals_x.push_back(avr_normals[0]);
//            cur_normals_y.push_back(avr_normals[1]);
//            std::cout<<avr_normals<<std::endl;
//            if(avr_normals<=0.01)
//            {
//                cur_cloud_normals.push_back(pcl::PointCloud<pcl::PointNormal>::Ptr (new pcl::PointCloud<pcl::PointNormal>));
//                //    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
//                pcl::concatenateFields (*cur_cloud.at(cloud_i), *cloud_normals, *cur_cloud_normals.at(cur_cloud_normals.size()-1));
//                //    std::cout<<cloud_with_normals->at(0).curvature<<std::endl;
//                std::stringstream ss;
//                ss << root_str.str() <<"normals/" <<file_cnt <<'/' <<file_cnt <<'-'<<cloud_i << ".pcd";

//                pcl::io::savePCDFile(ss.str(), *cur_cloud_normals.at(cur_cloud_normals.size()-1));

//            }

//        }
//        //ICP****************************************************************************************************************************
        if(file_cnt!=0)
        {

            //fd.open("/home/wangzt/run/0_1.txt");
            for(int cur_i=0; cur_i<cur_cloud.size(); cur_i++)
            {
//                double c_nx;
//                double c_ny;
//                double c_nz;
//                for(int n_i=0; n_i<cur_cloud_normals.at(cur_i)->size(); n_i++)
//                {
//                    c_nx+=cur_cloud_normals.at(cur_i)->points[n_i].normal_x;
//                    c_ny+=cur_cloud_normals.at(cur_i)->points[n_i].normal_y;
//                    c_nz+=cur_cloud_normals.at(cur_i)->points[n_i].normal_z;
//                }
//                c_nx/=cur_cloud_normals.at(cur_i)->size();
//                c_ny/=cur_cloud_normals.at(cur_i)->size();
//                c_nz/=cur_cloud_normals.at(cur_i)->size();


                for(int pre_i=0; pre_i<pre_cloud.size(); pre_i++)
                {
//                    double p_nx;
//                    double p_ny;
//                    double p_nz;
//                    for(int n_i=0; n_i<pre_cloud_normals.at(pre_i)->size(); n_i++)
//                    {
//                        p_nx+=pre_cloud_normals.at(pre_i)->points[n_i].normal_x;
//                        p_ny+=pre_cloud_normals.at(pre_i)->points[n_i].normal_y;
//                        p_nz+=pre_cloud_normals.at(pre_i)->points[n_i].normal_z;
//                    }
//                    p_nx/=pre_cloud_normals.at(pre_i)->size();
//                    p_ny/=pre_cloud_normals.at(pre_i)->size();
//                    p_nz/=pre_cloud_normals.at(pre_i)->size();

//                    if(abs(c_nx-p_nx)<0.05&&abs(c_ny-p_ny)<0.05&&abs(c_nz-p_nz)<0.05)
//                    {

                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp (new pcl::PointCloud<pcl::PointXYZ>);
                        *cloud_icp = *pre_cloud.at(pre_i);
                        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                        icp.setMaximumIterations (3);
                        icp.setInputSource (cloud_icp);
                        icp.setInputTarget (cur_cloud.at(cur_i));
                        time.tic ();
                        icp.align (*cloud_icp);
                        std::cout <<"****"<<  time.toc () << " ms" << std::endl;
                        if (icp.hasConverged ())
                        {
                            std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                            std::cout << "\nICP transformation " << '1' << " : cloud_icp -> cloud_in" << std::endl;
                            transformation_matrix = icp.getFinalTransformation ().cast<double>();
                            std::cout<<"pre:"<<pre_i<<", cur:"<<cur_i<<std::endl;


                            double r = (180/3.1415926)*atan2(transformation_matrix(1, 0),transformation_matrix(0, 0));
                            fd<<"pre:"<<pre_i<<", cur:"<<cur_i<<"  :"<<icp.getFitnessScore ()<<":"
                             <<transformation_matrix(0,3)<<","<<transformation_matrix(1,3)<<","<<r<<"   "<<std::endl;
//                            <<cur_normals_x.at(cur_i)<<","<<cur_normals_y.at(cur_i)<<std::endl;

                            print4x4Matrix (transformation_matrix);
                            if((r*r)<=10)
                            {
                                fd2.open("/home/wangzt/run/final.txt",std::ios::app);
                                fd2<<file_cnt<<": pre:"<<pre_i<<", cur:"<<cur_i<<":"<<icp.getFitnessScore ()<<":"
                                  <<transformation_matrix(0,3)<<","<<transformation_matrix(1,3)<<","<<r<<"   "<<std::endl;
                                   //<<cur_normals_x.at(cur_i)<<","<<cur_normals_y.at(cur_i)<<std::endl;
                                fd2.close();

                            }



                        }


//                    }


                }

            }
            fd.close();
//            fd2.open("/home/wangzt/run/final2.txt",std::ios::app);
//            fd2<<time.toc()<<std::endl;
//            fd2.close();

        }
        pre_cloud.clear();
        pre_cloud.assign(cur_cloud.begin(),cur_cloud.end());
//        pre_normals_x.clear();
//        pre_normals_y.clear();
//        pre_normals_x.assign(cur_normals_x.begin(),cur_normals_x.end());
//        pre_normals_y.assign(cur_normals_y.begin(),cur_normals_y.end());
   //     std::cout<<pre_cloud_normals.size()<<std::endl;
    //    cur_cloud_normals.clear();
        cur_cloud.clear();
//        cur_normals_x.clear();
//        cur_normals_y.clear();
//        std::cout <<  time.toc () << " ms" << std::endl;
        //    system("pcl_viewer /home/wangzt/run/normals/10/*");

        //************************************************************************************************************************************
        file_cnt+=1;

    }
    return 0;
}
