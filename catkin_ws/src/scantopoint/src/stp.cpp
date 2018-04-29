#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

/*make a class to handle the transformation of LRF data*/

class My_Filter {
     public:
        My_Filter();																	//convert the LaserScan message to PointCloud2
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);                //the callback function of scan message
     private:
        ros::NodeHandle node_;															//initiation of node's handle
        laser_geometry::LaserProjection projector_;										//initiate an object of the transformation class
        tf::TransformListener tfListener_;												//initiate an object of the tf subscribe class

        ros::Publisher point_cloud_publisher_;                                          //initiate an object of the publisher class
        ros::Subscriber scan_sub_;														//initiate an object of the subscriber class
};

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 200, &My_Filter::scanCallback, this);  //subscribe to the topic "/scan"
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud_in", 200, false);		 //publish the pointcloud2 message on topic"cloud_in"
        //tfListener_.setExtrapolationLimit(ros::Duration(0.1));												 //0.1s wait for the tf message
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
//    if(!tfListener_.waitForTransform(scan->header.frame_id,"/base_link",scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
//           ros::Duration(0.10))){
//        return;
//     }
   // projector_.transformLaserScanToPointCloud(*scan, cloud); 						 //transform the scan message to pointcloud2
    projector_.projectLaser(*scan, cloud);
    point_cloud_publisher_.publish(cloud);																	 //publish the pointcloud2
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stp");

    My_Filter filter;

//    sensor_msgs::PointCloud2 pc2;

    ros::spin();

    return 0;
}

